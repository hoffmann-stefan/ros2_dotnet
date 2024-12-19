/* Copyright 2024 Stefan Hoffmann <stefan.hoffmann@4am-robotics.com>
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
using rcl_interfaces.msg;
using ROS2.ParameterHandling.Exceptions;
using ParameterTypeMsg = rcl_interfaces.msg.ParameterType;
using ParameterMsg = rcl_interfaces.msg.Parameter;

namespace ROS2
{
    public enum ParameterType
    {
        NotSet = ParameterTypeMsg.PARAMETER_NOT_SET,
        Bool = ParameterTypeMsg.PARAMETER_BOOL,
        Integer = ParameterTypeMsg.PARAMETER_INTEGER,
        Double = ParameterTypeMsg.PARAMETER_DOUBLE,
        String = ParameterTypeMsg.PARAMETER_STRING,
        ByteArray = ParameterTypeMsg.PARAMETER_BYTE_ARRAY,
        BoolArray = ParameterTypeMsg.PARAMETER_BOOL_ARRAY,
        IntegerArray = ParameterTypeMsg.PARAMETER_INTEGER_ARRAY,
        DoubleArray = ParameterTypeMsg.PARAMETER_DOUBLE_ARRAY,
        StringArray = ParameterTypeMsg.PARAMETER_STRING_ARRAY
    }

    public sealed class Parameter
    {
        private readonly object _value;

        // Only provide public Create methods.
        // This allows to introduce Parameter<T> later on if needed.
        // Constructors can't return subtypes, but create methods can.
        private Parameter(string name, ParameterType type, object value)
        {
            Name = name;
            Type = type;
            _value = value ?? throw new ArgumentNullException(nameof(value));
        }

        public string Name { get; }
        public ParameterType Type { get; }

        public Parameter Create<T>(string name, T value)
        {
            if (typeof(T) == typeof(bool))
            {
                return new Parameter(name, ParameterType.Bool, value);
            }
            else if (typeof(T) == typeof(long))
            {
                return new Parameter(name, ParameterType.Integer, value);
            }
            else if (typeof(T) == typeof(double))
            {
                return new Parameter(name, ParameterType.Double, value);
            }
            else if (typeof(T) == typeof(string))
            {
                return new Parameter(name, ParameterType.String, value);
            }
            else if (typeof(T) == typeof(List<byte>))
            {
                return new Parameter(name, ParameterType.ByteArray, value);
            }
            else if (typeof(T) == typeof(List<bool>))
            {
                return new Parameter(name, ParameterType.BoolArray, value);
            }
            else if (typeof(T) == typeof(List<long>))
            {
                return new Parameter(name, ParameterType.IntegerArray, value);
            }
            else if (typeof(T) == typeof(List<double>))
            {
                return new Parameter(name, ParameterType.DoubleArray, value);
            }
            else if (typeof(T) == typeof(List<string>))
            {
                return new Parameter(name, ParameterType.StringArray, value);
            }
            else
            {
                throw new InvalidParameterTypeException(typeof(T));
            }
        }

        public static Parameter Create(string name, bool value)
            => new Parameter(name, ParameterType.Bool, value);

        public static Parameter Create(string name, long value)
            => new Parameter(name, ParameterType.Integer, value);

        public static Parameter Create(string name, double value)
            => new Parameter(name, ParameterType.Double, value);

        public static Parameter Create(string name, string value)
            => new Parameter(name, ParameterType.String, value);

        public static Parameter Create(string name, List<byte> value)
            => new Parameter(name, ParameterType.ByteArray, value);

        public static Parameter Create(string name, List<bool> value)
            => new Parameter(name, ParameterType.BoolArray, value);

        public static Parameter Create(string name, List<long> value)
            => new Parameter(name, ParameterType.IntegerArray, value);

        public static Parameter Create(string name, List<double> value)
            => new Parameter(name, ParameterType.DoubleArray, value);

        public static Parameter Create(string name, List<string> value)
            => new Parameter(name, ParameterType.StringArray, value);

        public static Parameter CreateFromMessage(ParameterMsg msg)
        {
            switch (msg.Value.Type)
            {
                case ParameterTypeMsg.PARAMETER_BOOL:
                    return new Parameter(msg.Name, ParameterType.Bool, msg.Value.BoolValue);
                case ParameterTypeMsg.PARAMETER_INTEGER:
                    return new Parameter(msg.Name, ParameterType.Integer, msg.Value.IntegerValue);
                case ParameterTypeMsg.PARAMETER_DOUBLE:
                    return new Parameter(msg.Name, ParameterType.Double, msg.Value.DoubleValue);
                case ParameterTypeMsg.PARAMETER_STRING:
                    return new Parameter(msg.Name, ParameterType.String, msg.Value.StringValue);
                case ParameterTypeMsg.PARAMETER_BYTE_ARRAY:
                    return new Parameter(msg.Name, ParameterType.ByteArray, msg.Value.ByteArrayValue);
                case ParameterTypeMsg.PARAMETER_BOOL_ARRAY:
                    return new Parameter(msg.Name, ParameterType.BoolArray, msg.Value.BoolArrayValue);
                case ParameterTypeMsg.PARAMETER_INTEGER_ARRAY:
                    return new Parameter(msg.Name, ParameterType.IntegerArray, msg.Value.IntegerArrayValue);
                case ParameterTypeMsg.PARAMETER_DOUBLE_ARRAY:
                    return new Parameter(msg.Name, ParameterType.DoubleArray, msg.Value.DoubleArrayValue);
                case ParameterTypeMsg.PARAMETER_STRING_ARRAY:
                    return new Parameter(msg.Name, ParameterType.StringArray, msg.Value.StringArrayValue);
                default:
                    throw new InvalidParameterTypeException(msg.Value.Type);
            }
        }

        internal static Parameter CreateFromMessageDeepCopy(ParameterMsg msg)
        {
            switch (msg.Value.Type)
            {
                case ParameterTypeMsg.PARAMETER_BOOL:
                    return new Parameter(msg.Name, ParameterType.Bool, msg.Value.BoolValue);
                case ParameterTypeMsg.PARAMETER_INTEGER:
                    return new Parameter(msg.Name, ParameterType.Integer, msg.Value.IntegerValue);
                case ParameterTypeMsg.PARAMETER_DOUBLE:
                    return new Parameter(msg.Name, ParameterType.Double, msg.Value.DoubleValue);
                case ParameterTypeMsg.PARAMETER_STRING:
                    return new Parameter(msg.Name, ParameterType.String, msg.Value.StringValue);
                case ParameterTypeMsg.PARAMETER_BYTE_ARRAY:
                    return new Parameter(msg.Name, ParameterType.ByteArray, msg.Value.ByteArrayValue.ToList());
                case ParameterTypeMsg.PARAMETER_BOOL_ARRAY:
                    return new Parameter(msg.Name, ParameterType.BoolArray, msg.Value.BoolArrayValue.ToList());
                case ParameterTypeMsg.PARAMETER_INTEGER_ARRAY:
                    return new Parameter(msg.Name, ParameterType.IntegerArray, msg.Value.IntegerArrayValue.ToList());
                case ParameterTypeMsg.PARAMETER_DOUBLE_ARRAY:
                    return new Parameter(msg.Name, ParameterType.DoubleArray, msg.Value.DoubleArrayValue.ToList());
                case ParameterTypeMsg.PARAMETER_STRING_ARRAY:
                    return new Parameter(msg.Name, ParameterType.StringArray, msg.Value.StringArrayValue.ToList());
                default:
                    throw new InvalidParameterTypeException(msg.Value.Type);
            }
        }

        public T Get<T>()
        {
            if (_value is T value)
            {
                return value;
            }

            throw new InvalidCastException($"Parameter value is not of type {typeof(T).Name}");
        }

        public bool TryGet<T>(out T value)
        {
            if (_value is T castValue)
            {
                value = castValue;
                return true;
            }

            value = default;
            return false;
        }

        public bool GetBool() => Get<bool>();
        public bool TryGetBool(out bool value) => TryGet(out value);

        public long GetInteger() => Get<long>();
        public bool TryGetInteger(out long value) => TryGet(out value);

        public double GetDouble() => Get<double>();
        public bool TryGetDouble(out double value) => TryGet(out value);

        public string GetString() => Get<string>();
        public bool TryGetString(out string value) => TryGet(out value);

        public List<byte> GetByteArray() => Get<List<byte>>();
        public bool TryGetByteArray(out List<byte> value) => TryGet(out value);

        public List<bool> GetBoolArray() => Get<List<bool>>();
        public bool TryGetBoolArray(out List<bool> value) => TryGet(out value);

        public List<long> GetIntegerArray() => Get<List<long>>();
        public bool TryGetIntegerArray(out List<long> value) => TryGet(out value);

        public List<double> GetDoubleArray() => Get<List<double>>();
        public bool TryGetDoubleArray(out List<double> value) => TryGet(out value);

        public List<string> GetStringArray() => Get<List<string>>();
        public bool TryGetStringArray(out List<string> value) => TryGet(out value);

        public ParameterMsg ToMessage()
        {
            ParameterMsg msg = new ParameterMsg
            {
                Name = Name,
                Value = new ParameterValue { Type = (byte)Type }
            };

            switch (Type)
            {
                case ParameterType.NotSet:
                    break;
                case ParameterType.Bool:
                    msg.Value.BoolValue = (bool)_value;
                    break;
                case ParameterType.Integer:
                    msg.Value.IntegerValue = (long)_value;
                    break;
                case ParameterType.Double:
                    msg.Value.DoubleValue = (double)_value;
                    break;
                case ParameterType.String:
                    msg.Value.StringValue = (string)_value;
                    break;
                case ParameterType.ByteArray:
                    msg.Value.ByteArrayValue = (List<byte>)_value;
                    break;
                case ParameterType.BoolArray:
                    msg.Value.BoolArrayValue = (List<bool>)_value;
                    break;
                case ParameterType.IntegerArray:
                    msg.Value.IntegerArrayValue = (List<long>)_value;
                    break;
                case ParameterType.DoubleArray:
                    msg.Value.DoubleArrayValue = (List<double>)_value;
                    break;
                case ParameterType.StringArray:
                    msg.Value.StringArrayValue = (List<string>)_value;
                    break;
                default:
                    // Can't happen as no other types are passed to the private constructor.
                    throw new InvalidParameterTypeException((byte)Type);
            }

            return msg;
        }

        internal ParameterMsg ToMessageDeepCopy()
        {
            ParameterMsg msg = new ParameterMsg
            {
                Name = Name,
                Value = new ParameterValue { Type = (byte)Type }
            };

            switch (Type)
            {
                case ParameterType.NotSet:
                    break;
                case ParameterType.Bool:
                    msg.Value.BoolValue = (bool)_value;
                    break;
                case ParameterType.Integer:
                    msg.Value.IntegerValue = (long)_value;
                    break;
                case ParameterType.Double:
                    msg.Value.DoubleValue = (double)_value;
                    break;
                case ParameterType.String:
                    msg.Value.StringValue = (string)_value;
                    break;
                case ParameterType.ByteArray:
                    msg.Value.ByteArrayValue.AddRange((List<byte>)_value);
                    break;
                case ParameterType.BoolArray:
                    msg.Value.BoolArrayValue.AddRange((List<bool>)_value);
                    break;
                case ParameterType.IntegerArray:
                    msg.Value.IntegerArrayValue.AddRange((List<long>)_value);
                    break;
                case ParameterType.DoubleArray:
                    msg.Value.DoubleArrayValue.AddRange((List<double>)_value);
                    break;
                case ParameterType.StringArray:
                    msg.Value.StringArrayValue.AddRange((List<string>)_value);
                    break;
                default:
                    // Can't happen as no other types are passed to the private constructor.
                    throw new InvalidParameterTypeException((byte)Type);
            }

            return msg;
        }
    }
}
