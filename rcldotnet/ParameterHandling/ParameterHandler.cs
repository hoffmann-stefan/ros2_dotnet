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
    internal class ParameterHandler
    {
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
                if (_descriptors.TryGetValue(name, out ParameterDescriptor descriptor))
                {
                    response.Descriptors.Add(descriptor);
                }
                else
                {
                    // return an empty list if one parameter is not declared.
                    response.Descriptors.Clear();
                    return;
                }
            }
        }

        private void OnGetParameterTypesServiceRequest(GetParameterTypes_Request request, GetParameterTypes_Response response)
        {
            foreach (string name in request.Names)
            {
                if (_parameters.TryGetValue(name, out ParameterMsg parameter))
                {
                    response.Types.Add(parameter.Value.Type);
                }
                else
                {
                    // return an empty list if one parameter is not declared.
                    response.Types.Clear();
                    return;
                }
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
                else
                {
                    // return an empty list if one parameter is not declared.
                    response.Values.Clear();
                    return;
                }
            }
        }

        private void OnListParametersServiceRequest(ListParameters_Request request, ListParameters_Response response)
        {
            try
            {
                if (request.Depth >= 0 && request.Depth <= int.MaxValue)
                {
                    response.Result = ListParameters(request.Prefixes, (int)request.Depth);
                }
                else
                {
                    // return an empty list if the depth is too large.
                }
            }
            catch (Exception)
            {
                response.Result.Names.Clear();
                response.Result.Prefixes.Clear();
            }
        }

        private void OnSetParametersServiceRequest(SetParameters_Request request, SetParameters_Response response)
        {
            foreach (ParameterMsg source in request.Parameters)
            {
                SetParametersResult result;
                try
                {
                    result = SetParametersAtomically(new List<ParameterMsg> { source });
                }
                catch (Exception ex)
                {
                    result = new SetParametersResult
                    {
                        Successful = false,
                        Reason = ex.Message
                    };
                }

                response.Results.Add(result);
            }
        }

        private void OnSetParametersAtomicallyServiceRequest(SetParametersAtomically_Request request, SetParametersAtomically_Response response)
        {
            SetParametersResult result;
            try
            {
                result = SetParametersAtomically(request.Parameters);
            }
            catch (Exception ex)
            {
                result = new SetParametersResult
                {
                    Successful = false,
                    Reason = ex.Message
                };
            }

            response.Result = result;
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

        private bool TryGetParameterOverride(string name, ParameterValue parameterOverride)
        {
            if (!RCLdotnet.HasGlobalParameterOverrides) return false;

            bool overrideExists = false;

            using (SafeHandle messageHandle = MessageStaticMemberCache<ParameterValue>.CreateMessageHandle())
            {
                RCLdotnet.WriteToMessageHandle(parameterOverride, messageHandle);
                overrideExists = ParameterDelegates.native_rcl_try_get_parameter(messageHandle, RCLdotnet.GlobalParameterOverrideHandle, _node.Handle, name) != 0;
                if (overrideExists)
                {
                    // Avoid reading from the message handle if the parameter is
                    // not overridden as null checks or failed allocations in
                    // native_rcl_try_get_parameter return false as well.
                    RCLdotnet.ReadFromMessageHandle(parameterOverride, messageHandle);
                }
            }

            return overrideExists;
        }

        private Parameter DeclareParameter(string name, byte typeCode, Action<ParameterValue> assignDefaultCallback, ParameterDescriptor descriptor = null)
        {
            if (descriptor == null)
            {
                descriptor = new ParameterDescriptor
                {
                    Name = name,
                    Type = typeCode
                };
            }
            else
            {
                // Copy the descriptor to avoid mutation of the internal storage via mutable object.
                descriptor = RCLdotnet.DeepCopyMessage(descriptor);

                // Override the name and type in the provided descriptor.
                // rclpy and rclcpp do this in here:
                // - https://github.com/ros2/rclpy/blob/4e8b071127228d5dace5aebf61d02260ecb91253/rclpy/rclpy/node.py#L1303
                // - https://github.com/ros2/rclcpp/blob/a0a2a067d84fd6a38ab4f71b691d51ca5aa97ba5/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L460
                // - https://github.com/ros2/rclcpp/blob/a0a2a067d84fd6a38ab4f71b691d51ca5aa97ba5/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L287-L289
                descriptor.Name = name;
                descriptor.Type = typeCode;
            }

            // This check doesn't happen atomically, but so is it in rclcpp and
            // rclpy as far as I can see as well. Maybe check if this should be
            // changed (upstream as well).
            if (_parameters.TryGetValue(name, out ParameterMsg parameter))
            {
                throw new ParameterAlreadyDeclaredException(name);
            }

            ParameterValue declaredValue = new ParameterValue { Type = typeCode };
            ParameterMsg declaredParameter = new ParameterMsg { Name = name, Value = declaredValue };
            if (!TryGetParameterOverride(name, declaredValue))
            {
                // The callback assures the provided value is copied to avoid
                // mutation of the internal parameter values via mutable
                // objects.
                assignDefaultCallback?.Invoke(declaredParameter.Value);
            }

            _parameters.Add(name, declaredParameter);
            _descriptors.Add(name, descriptor);

            PublishParametersDeclaredEvent(new List<ParameterMsg> { declaredParameter });

            // Copy into local variable to avoid races when another thread deregisters the last callback.
            var postSetParameterCallbacks = _postSetParameterCallbacks;
            if (postSetParameterCallbacks != null)
            {
                List<Parameter> parameterObjects = new List<Parameter> { Parameter.CreateFromMessageDeepCopy(declaredParameter) };
                postSetParameterCallbacks.Invoke(parameterObjects);
            }

            return Parameter.CreateFromMessageDeepCopy(declaredParameter);
        }

        public Parameter DeclareParameter(string name, bool defaultValue, ParameterDescriptor descriptor = null)
        {
            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_BOOL, value => { value.BoolValue = defaultValue; }, descriptor);
        }

        public Parameter DeclareParameter(string name, long defaultValue, ParameterDescriptor descriptor = null)
        {
            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_INTEGER, value => { value.IntegerValue = defaultValue; }, descriptor);
        }

        public Parameter DeclareParameter(string name, double defaultValue, ParameterDescriptor descriptor = null)
        {
            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_DOUBLE, value => { value.DoubleValue = defaultValue; }, descriptor);
        }

        public Parameter DeclareParameter(string name, string defaultValue, ParameterDescriptor descriptor = null)
        {
            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_STRING, value => { value.StringValue = defaultValue; }, descriptor);
        }

        public Parameter DeclareParameter(string name, IEnumerable<byte> defaultValue, ParameterDescriptor descriptor = null)
        {
            if (defaultValue == null) throw new ArgumentNullException(nameof(defaultValue));

            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_BYTE_ARRAY, value =>
            {
                value.ByteArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public Parameter DeclareParameter(string name, IEnumerable<bool> defaultValue, ParameterDescriptor descriptor = null)
        {
            if (defaultValue == null) throw new ArgumentNullException(nameof(defaultValue));

            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_BOOL_ARRAY, value =>
            {
                value.BoolArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public Parameter DeclareParameter(string name, IEnumerable<long> defaultValue, ParameterDescriptor descriptor = null)
        {
            if (defaultValue == null) throw new ArgumentNullException(nameof(defaultValue));

            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_INTEGER_ARRAY, value =>
            {
                value.IntegerArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public Parameter DeclareParameter(string name, IEnumerable<double> defaultValue, ParameterDescriptor descriptor = null)
        {
            if (defaultValue == null) throw new ArgumentNullException(nameof(defaultValue));

            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_DOUBLE_ARRAY, value =>
            {
                value.DoubleArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public Parameter DeclareParameter(string name, IEnumerable<string> defaultValue, ParameterDescriptor descriptor = null)
        {
            if (defaultValue == null) throw new ArgumentNullException(nameof(defaultValue));

            return DeclareParameter(name, ParameterTypeMsg.PARAMETER_STRING_ARRAY, value =>
            {
                value.StringArrayValue.AddRange(defaultValue);
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

            // TODO: Check again in rclcpp and rclpy if undeclaring a parameter
            // should send an event.
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
                else
                {
                    throw new ParameterNotDeclaredException(parameterName);
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
                throw new ParameterNotDeclaredException(update.Name);
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

        public ListParametersResult ListParameters(List<string> prefixes, int depth)
        {
            if (depth < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(depth), "Depth must be greater than or equal to 0.");
            }

            const char Separator = '.';
            bool SeparatorsLessThanDepth(string str) => str.Count(c => c == Separator) < depth;

            bool recursive = (prefixes.Count == 0) && (ulong)depth == ListParameters_Request.DEPTH_RECURSIVE;

            var result = new ListParametersResult();
            foreach (var kv in _parameters)
            {
                if (!recursive)
                {
                    bool getAll = (prefixes.Count == 0) && SeparatorsLessThanDepth(kv.Key);
                    if (!getAll)
                    {
                        bool prefixMatches = prefixes.Any(prefix =>
                        {
                            if (kv.Key == prefix)
                            {
                                return true;
                            }

                            if (kv.Key.StartsWith(prefix + Separator))
                            {
                                if ((ulong)depth == ListParameters_Request.DEPTH_RECURSIVE)
                                {
                                    return true;
                                }

                                var substr = kv.Key.Substring(prefix.Length + 1);
                                return SeparatorsLessThanDepth(substr);
                            }

                            return false;
                        });

                        if (!prefixMatches)
                        {
                            continue;
                        }
                    }
                }

                result.Names.Add(kv.Key);
                var lastSeparator = kv.Key.LastIndexOf(Separator);
                if (lastSeparator != -1)
                {
                    var prefix = kv.Key.Substring(0, lastSeparator);
                    if (!result.Prefixes.Contains(prefix))
                    {
                        result.Prefixes.Add(prefix);
                    }
                }
            }

            return result;
        }

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
