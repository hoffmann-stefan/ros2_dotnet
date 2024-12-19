/* Copyright 2023 Queensland University of Technology.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using rcl_interfaces.msg;
using rcl_interfaces.srv;
using ROS2.ParameterHandling;
using ROS2.ParameterHandling.Exceptions;
using ParameterTypeMsg = rcl_interfaces.msg.ParameterType;
using ParameterMsg = rcl_interfaces.msg.Parameter;

namespace ROS2
{
    public class ParameterHandler
    {
        private static readonly IDictionary<Type, byte> _typeToParameterType = new Dictionary<Type, byte>
        {
            {typeof(bool), ParameterTypeMsg.PARAMETER_BOOL},
            {typeof(long), ParameterTypeMsg.PARAMETER_INTEGER},
            {typeof(double), ParameterTypeMsg.PARAMETER_DOUBLE},
            {typeof(string), ParameterTypeMsg.PARAMETER_STRING},
            {typeof(List<byte>), ParameterTypeMsg.PARAMETER_BYTE_ARRAY},
            {typeof(List<bool>), ParameterTypeMsg.PARAMETER_BOOL_ARRAY},
            {typeof(List<long>), ParameterTypeMsg.PARAMETER_INTEGER_ARRAY},
            {typeof(List<double>), ParameterTypeMsg.PARAMETER_DOUBLE_ARRAY},
            {typeof(List<string>), ParameterTypeMsg.PARAMETER_STRING_ARRAY}
        };

        private readonly Node _node;
        private readonly IDictionary<string, ParameterMsg> _parameters = new Dictionary<string, ParameterMsg>();
        private readonly IDictionary<string, ParameterDescriptor> _descriptors = new Dictionary<string, ParameterDescriptor>();

        private readonly Publisher<ParameterEvent> _publisherEvent;

        private Action<List<Parameter>> _postSetParameterCallbacks;

        internal ParameterHandler(Node node)
        {
            _node = node;
            _publisherEvent = node.CreatePublisher<ParameterEvent>("/parameter_events", QosProfile.ParameterEventsProfile);

            node.CreateService<DescribeParameters, DescribeParameters_Request, DescribeParameters_Response>("~/describe_parameters", OnDescribeParametersServiceRequest);
            node.CreateService<GetParameterTypes, GetParameterTypes_Request, GetParameterTypes_Response>("~/get_parameter_types", OnGetParameterTypesServiceRequest);
            node.CreateService<GetParameters, GetParameters_Request, GetParameters_Response>("~/get_parameters", OnGetParametersServiceRequest);
            node.CreateService<ListParameters, ListParameters_Request, ListParameters_Response>("~/list_parameters", OnListParametersServiceRequest);
            node.CreateService<SetParameters, SetParameters_Request, SetParameters_Response>("~/set_parameters", OnSetParametersServiceRequest);
            node.CreateService<SetParametersAtomically, SetParametersAtomically_Request, SetParametersAtomically_Response>("~/set_parameters_atomically", OnSetParametersAtomicallyServiceRequest);
        }

        #region Service Request Handlers

        private void OnDescribeParametersServiceRequest(DescribeParameters_Request request, DescribeParameters_Response response)
        {
            foreach (string name in request.Names)
            {
                response.Descriptors.Add(
                    _descriptors.TryGetValue(name, out ParameterDescriptor descriptor)
                        ? descriptor
                        : new ParameterDescriptor());
            }
        }

        private void OnGetParameterTypesServiceRequest(GetParameterTypes_Request request, GetParameterTypes_Response response)
        {
            foreach (ParameterMsg parameter in _parameters.Values)
            {
                response.Types.Add(parameter.Value.Type);
            }
        }

        private void OnGetParametersServiceRequest(GetParameters_Request request, GetParameters_Response response)
        {
            foreach (string parameterName in request.Names)
            {
                if (_parameters.TryGetValue(parameterName, out ParameterMsg parameter))
                {
                    response.Values.Add(parameter.Value);
                }
            }
        }

        private void OnListParametersServiceRequest(ListParameters_Request request, ListParameters_Response response)
        {
            bool hasPrefixes = request.Prefixes.Count != 0;
            foreach (ParameterMsg parameter in _parameters.Values)
            {
                bool matchesCriteria = !hasPrefixes;

                if (hasPrefixes)
                {
                    foreach (string prefix in request.Prefixes)
                    {
                        if (parameter.Name.StartsWith(prefix))
                        {
                            matchesCriteria = true;
                            break;
                        }
                    }
                }

                if (matchesCriteria) response.Result.Names.Add(parameter.Name);
            }
        }

        private void OnSetParametersServiceRequest(SetParameters_Request request, SetParameters_Response response)
        {
            foreach (ParameterMsg source in request.Parameters)
            {
                response.Results.Add(SetParametersAtomically(new List<ParameterMsg> { source }));
            }
        }

        private void OnSetParametersAtomicallyServiceRequest(SetParametersAtomically_Request request, SetParametersAtomically_Response response)
        {
            response.Result = SetParametersAtomically(request.Parameters);
        }

        #endregion

        public IDisposable AddPostSetParameterCallback(Action<List<Parameter>> callback)
        {
            var disposable = new PostSetParameterDisposable(this, callback);
            _postSetParameterCallbacks += callback;
            return disposable;
        }

        internal void RemovePostSetParameterCallback(Action<List<Parameter>> callback)
        {
            _postSetParameterCallbacks -= callback;
        }

        private ParameterEvent GenerateParameterEventMessage()
        {
            return new ParameterEvent
            {
                Node = _node.FullyQualifiedName,
                Stamp = _node.Clock.Now()
            };
        }

        private void PublishParametersDeclaredEvent(List<ParameterMsg> parameters)
        {
            ParameterEvent parameterEvent = GenerateParameterEventMessage();
            parameterEvent.NewParameters.AddRange(parameters);
            _publisherEvent.Publish(parameterEvent);
        }

        private void PublishParametersChangedEvent(IEnumerable<ParameterMsg> parameters)
        {
            ParameterEvent parameterEvent = GenerateParameterEventMessage();
            parameterEvent.ChangedParameters.AddRange(parameters);
            _publisherEvent.Publish(parameterEvent);
        }

        private void PublishParametersDeletedEvent(IEnumerable<ParameterMsg> parameters)
        {
            ParameterEvent parameterEvent = GenerateParameterEventMessage();
            parameterEvent.DeletedParameters.AddRange(parameters);
            _publisherEvent.Publish(parameterEvent);
        }

        private bool TryGetParameterOverride(string name, ref ParameterValue parameterOverride)
        {
            if (!RCLdotnet.HasGlobalParameterOverrides) return false;

            bool overrideExists = false;

            using (SafeHandle messageHandle = MessageStaticMemberCache<ParameterValue>.CreateMessageHandle())
            {
                RCLdotnet.WriteToMessageHandle(parameterOverride, messageHandle);
                overrideExists = ParameterDelegates.native_rcl_try_get_parameter(messageHandle, RCLdotnet.GlobalParameterOverrideHandle, _node.Handle, name) != 0;
                RCLdotnet.ReadFromMessageHandle(parameterOverride, messageHandle);
            }

            return overrideExists;
        }

        private void DeclareParameter(string name, Type type, Action<ParameterValue> assignDefaultCallback, ParameterDescriptor descriptor = null)
        {
            if (!_typeToParameterType.TryGetValue(type, out byte typeCode))
            {
                throw new InvalidParameterTypeException(type);
            }

            if (descriptor == null)
            {
                descriptor = new ParameterDescriptor
                {
                    Name = name,
                    Type = typeCode
                };
            }

            if (_parameters.TryGetValue(name, out ParameterMsg parameter))
            {

                if (parameter.Value.Type != typeCode)
                {
                    throw new ParameterTypeMismatchException(
                        $"Attempted to redefine parameter \"{name}\" from type {parameter.Value.Type} to {typeCode}!");
                }

                // TODO: Should we update the description if it doesn't match or throw an error?
                return;
            }

            ParameterValue declaredValue = new ParameterValue { Type = typeCode };
            ParameterMsg declaredParameter = new ParameterMsg { Name = name, Value = declaredValue };
            if (!TryGetParameterOverride(name, ref declaredValue))
            {
                // The callback assures the provided value is copied to avoid
                // mutation of the internal parameter values via mutable
                // objects.
                assignDefaultCallback?.Invoke(declaredParameter.Value);
            }

            _parameters.Add(name, declaredParameter);
            _descriptors.Add(name, descriptor);

            PublishParametersDeclaredEvent(new List<ParameterMsg> { declaredParameter });

            List<Parameter> parameterObjects = new List<Parameter> { Parameter.CreateFromMessage(declaredParameter) };
            _postSetParameterCallbacks?.Invoke(parameterObjects);
        }

        public void DeclareParameter(string name, bool defaultValue = false, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(bool), value => { value.BoolValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, int defaultValue = 0, ParameterDescriptor descriptor = null) => DeclareParameter(name, (long)defaultValue, descriptor);

        public void DeclareParameter(string name, long defaultValue = 0L, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(long), value => { value.IntegerValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, float defaultValue = 0.0f, ParameterDescriptor descriptor = null) => DeclareParameter(name, (double)defaultValue, descriptor);

        public void DeclareParameter(string name, double defaultValue = 0.0, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(double), value => { value.DoubleValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, string defaultValue = "", ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(string), value => { value.StringValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<byte> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<byte>), value =>
            {
                if (defaultValue != null) value.ByteArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<bool> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<bool>), value =>
            {
                if (defaultValue != null) value.BoolArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<long> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<long>), value =>
            {
                if (defaultValue != null) value.IntegerArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<double> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<double>), value =>
            {
                if (defaultValue != null) value.DoubleArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<string> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<string>), value =>
            {
                if (defaultValue != null) value.StringArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void UndeclareParameter(string name)
        {
            if (!_descriptors.TryGetValue(name, out ParameterDescriptor descriptor))
            {
                throw new ParameterNotDeclaredException(name);
            }

            if (descriptor.ReadOnly) throw new ParameterImmutableException(name);

            ParameterMsg parameter = _parameters[name];

            _parameters.Remove(name);
            _descriptors.Remove(name);

            PublishParametersDeletedEvent(new List<ParameterMsg> { parameter });
        }

        public Parameter GetParameter(string name)
        {
            if (_parameters.TryGetValue(name, out ParameterMsg parameter))
            {
                // Do a deep copy the parameter here to avoid mutation of the
                // internal parameter values by the caller.
                return Parameter.CreateFromMessageDeepCopy(parameter);
            }

            throw new ParameterNotDeclaredException(name);
        }

        public List<Parameter> GetParameters(IEnumerable<string> names)
        {
            List<Parameter> results = new List<Parameter>();

            foreach (string parameterName in names)
            {
                if (_parameters.TryGetValue(parameterName, out ParameterMsg parameter))
                {
                    // Do a deep copy the parameter here to avoid mutation of the
                    // internal parameter values by the caller.
                    results.Add(Parameter.CreateFromMessageDeepCopy(parameter));
                }
            }

            return results;
        }

        public SetParametersResult SetParameter(Parameter parameter)
        {
            // Do a deep copy the parameter here to avoid mutation of the
            // internal parameter values by the caller.
            var convertedParameters = new List<ParameterMsg> { parameter.ToMessageDeepCopy() };
            return SetParametersAtomically(convertedParameters);
        }

        public List<SetParametersResult> SetParameters(List<Parameter> parameters)
        {
            List<SetParametersResult> results = new List<SetParametersResult>();

            foreach (Parameter source in parameters)
            {
                // Do a deep copy the parameter here to avoid mutation of the
                // internal parameter values by the caller.
                var convertedParameters = new List<ParameterMsg> { source.ToMessageDeepCopy() };
                results.Add(SetParametersAtomically(convertedParameters));
            }

            return results;
        }

        public SetParametersResult SetParametersAtomically(List<Parameter> parameters)
        {
            // Do a deep copy the parameter here to avoid mutation of the
            // internal parameter values by the caller.
            var convertedParameters = parameters
                .Select(parameter => parameter.ToMessageDeepCopy())
                .ToList();

            return SetParametersAtomically(convertedParameters);
        }

        private SetParametersResult CheckParameterCompatibility(ParameterMsg update)
        {
            SetParametersResult result = new SetParametersResult();
            if (!_descriptors.TryGetValue(update.Name, out ParameterDescriptor descriptor))
            {
                result.Successful = false;
                result.Reason = "Parameter was not declared!";
            }
            else if (descriptor.ReadOnly)
            {
                result.Successful = false;
                result.Reason = "Parameter is read-only!";
            }
            else if (update.Value.Type != descriptor.Type)
            {
                result.Successful = false;
                result.Reason = $"Parameter type mismatch: {descriptor.Type} != {update.Value.Type}!";
            }
            // TODO: Check value compatibility against ParameterDescriptor constraints.
            else
            {
                result.Successful = true;
            }

            return result;
        }

        private void UpdateParameter(ParameterMsg source)
        {
            ParameterMsg target = _parameters[source.Name];

            // The lists are already deep copied in the public methods.
            switch (source.Value.Type)
            {
                case ParameterTypeMsg.PARAMETER_BOOL:
                    target.Value.BoolValue = source.Value.BoolValue;
                    break;
                case ParameterTypeMsg.PARAMETER_INTEGER:
                    target.Value.IntegerValue = source.Value.IntegerValue;
                    break;
                case ParameterTypeMsg.PARAMETER_DOUBLE:
                    target.Value.DoubleValue = source.Value.DoubleValue;
                    break;
                case ParameterTypeMsg.PARAMETER_STRING:
                    target.Value.StringValue = source.Value.StringValue;
                    break;
                case ParameterTypeMsg.PARAMETER_BYTE_ARRAY:
                    target.Value.ByteArrayValue = source.Value.ByteArrayValue;
                    break;
                case ParameterTypeMsg.PARAMETER_BOOL_ARRAY:
                    target.Value.BoolArrayValue = source.Value.BoolArrayValue;
                    break;
                case ParameterTypeMsg.PARAMETER_INTEGER_ARRAY:
                    target.Value.IntegerArrayValue = source.Value.IntegerArrayValue;
                    break;
                case ParameterTypeMsg.PARAMETER_DOUBLE_ARRAY:
                    target.Value.DoubleArrayValue = source.Value.DoubleArrayValue;
                    break;
                case ParameterTypeMsg.PARAMETER_STRING_ARRAY:
                    target.Value.StringArrayValue = source.Value.StringArrayValue;
                    break;
                default:
                    throw new InvalidParameterTypeException(source.Value.Type);
            }
        }

        private SetParametersResult SetParametersAtomically(List<ParameterMsg> parameters)
        {
            SetParametersResult result = new SetParametersResult();

            foreach (ParameterMsg source in parameters)
            {
                result = CheckParameterCompatibility(source);
                if (!result.Successful) break;
            }

            if (!result.Successful) return result;

            foreach (ParameterMsg source in parameters)
            {
                UpdateParameter(source);
            }

            PublishParametersChangedEvent(parameters);

            // Do a deep copy the parameters here to avoid mutation of the
            // internal values by the callback.
            List<Parameter> parameterObjects = parameters
                .Select(message => Parameter.CreateFromMessageDeepCopy(message))
                .ToList();

            _postSetParameterCallbacks?.Invoke(parameterObjects);

            return result;
        }

        public bool HasParameter(string name) => _parameters.ContainsKey(name);

        private class PostSetParameterDisposable : IDisposable
        {
            private readonly Action<List<Parameter>> _callback;
            private readonly ParameterHandler _handler;

            public PostSetParameterDisposable(ParameterHandler handler, Action<List<Parameter>> callback)
            {
                _handler = handler;
                _callback = callback;
            }

            public void Dispose()
            {
                _handler.RemovePostSetParameterCallback(_callback);
            }
        }
    }
}
