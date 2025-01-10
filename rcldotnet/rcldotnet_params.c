// Copyright 2023 Queensland University of Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rcl/rcl.h>
#include <rcl_yaml_param_parser/parser.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rcl_interfaces/msg/parameter_value.h>
#include <rcl_interfaces/msg/parameter_type.h>

#include "rcldotnet_params.h"

typedef struct rcl_void_array_s {
  /// Array with values
  void *values;
  /// Number of values in the array
  size_t size;
} rcl_void_array_t;

ROSIDL_RUNTIME_C__PRIMITIVE_SEQUENCE(void, void)

void native_rcl_destroy_rcl_params(void *rcl_params) {
  rcl_yaml_node_struct_fini((rcl_params_t *)rcl_params);
}

// function based of rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__copy
bool rcldotnet_params_copy_yaml_array_to_parameter_array(
  rosidl_runtime_c__void__Sequence *dest,
  const rcl_void_array_t *src,
  size_t element_size)
{
  if (!src || !dest) {
    return false;
  }
  if (dest->capacity < src->size) {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    void * data = (void *)allocator.reallocate(
      dest->data, element_size * src->size, allocator.state);
    if (!data) {
      return false;
    }
    dest->data = data;
    dest->capacity = src->size;
  }
  memcpy(dest->data, src->values, element_size * src->size);
  dest->size = src->size;
  return true;
}

// function based of rosidl_runtime_c__String__Sequence__init but modified to
// initialize elements from rcultis_string_array_t directly using
// rosidl_runtime_c__String__Sequence__init and then assigning each element in a
// loop afterwards would need additional memory allocation and deallocation (not
// that it maters that much here)
bool rcldotnet_params_copy_yaml_string_array_to_parameter_string_array(
  rosidl_runtime_c__String__Sequence *dest,
  rcutils_string_array_t *src)
{
  if (!dest || !src) {
    return false;
  }

  if (dest->data) {
    rosidl_runtime_c__String__Sequence__fini(dest);
  }

  size_t size = src->size;

  rosidl_runtime_c__String * data = NULL;
  if (size) {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data = (rosidl_runtime_c__String *)allocator.zero_allocate(
      size, sizeof(rosidl_runtime_c__String), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all sequence elements
    for (size_t i = 0; i < size; ++i) {
      if (!rosidl_runtime_c__String__assign(&(data[i]), src->data[i])) {
        /* free currently allocated and return false */
        for (; i-- > 0; ) {
          rosidl_runtime_c__String__fini(&data[i]);
        }
        allocator.deallocate(data, allocator.state);
        return false;
      }
    }
  }
  dest->data = data;
  dest->size = size;
  dest->capacity = size;
  return true;
}

bool rcldotnet_params_try_get_parameter_from_node_params(const rcl_node_params_t *node_params, const char *name, rcl_interfaces__msg__ParameterValue *param_value) {
  int param_index = 0;
  for (; param_index < node_params->num_params; param_index++) {
    if (strcmp(name, node_params->parameter_names[param_index]) == 0) {
      break;
    }
  }

  if (param_index >= node_params->num_params) {
    return false;
  }

  rcl_variant_t *rcl_param_value = &node_params->parameter_values[param_index];
  bool ret;

  // If the parameter is found, but the type from args or yaml does not match,
  // don't copy the value but return true to not use the default value specified
  // in declare parameters. This is how rclpy does it.
  // If null checks or allocations fail, return false.
  switch (param_value->type) {
    case rcl_interfaces__msg__ParameterType__PARAMETER_BOOL:
      if (rcl_param_value->bool_value == NULL) return true;
      param_value->bool_value = *rcl_param_value->bool_value;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER:
      if (rcl_param_value->integer_value == NULL) return true;
      param_value->integer_value = *rcl_param_value->integer_value;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE:
      if (rcl_param_value->double_value == NULL) return true;
      param_value->double_value = *rcl_param_value->double_value;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_STRING:
      if (rcl_param_value->string_value == NULL) return true;
      ret = rosidl_runtime_c__String__assign(&param_value->string_value, rcl_param_value->string_value);
      if (!ret) return false;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_BYTE_ARRAY:
      // Byte array parameter loading from YAML not implemented in RCL.
      //if (rcl_param_value->byte_array_value == NULL) return true;
      //ret = rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->byte_array_value, (rcl_void_array_t *)rcl_param_value->byte_array_value, sizeof(char));
      //if (!ret) return false;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_BOOL_ARRAY:
      if (rcl_param_value->bool_array_value == NULL) return true;
      ret = rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->bool_array_value, (rcl_void_array_t *)rcl_param_value->bool_array_value, sizeof(bool));
      if (!ret) return false;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER_ARRAY:
      if (rcl_param_value->integer_array_value == NULL) return true;
      ret = rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->integer_array_value, (rcl_void_array_t *)rcl_param_value->integer_array_value, sizeof(int64_t));
      if (!ret) return false;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE_ARRAY:
      if (rcl_param_value->double_array_value == NULL) return true;
      ret = rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->double_array_value, (rcl_void_array_t *)rcl_param_value->double_array_value, sizeof(double));
      if (!ret) return false;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_STRING_ARRAY:
      if (rcl_param_value->string_array_value == NULL) return true;
      ret = rcldotnet_params_copy_yaml_string_array_to_parameter_string_array(&param_value->string_array_value, rcl_param_value->string_array_value);
      if (!ret) return false;
    break;
  }

  return true;
}

int32_t /* bool */ native_rcl_try_get_parameter(void *param_value_handle, const void *params_handle, const void *node_handle, const char *name) {
  if (params_handle == NULL) return false;

  rcl_interfaces__msg__ParameterValue *param_value = (rcl_interfaces__msg__ParameterValue *)param_value_handle;
  const rcl_params_t *rcl_params = (const rcl_params_t *)params_handle;
  const rcl_node_t *node = (const rcl_node_t *)node_handle;
  const char *node_name = rcl_node_get_fully_qualified_name(node);

  // First check if there is an override which matches the fully qualified node name.
  for (int i = 0; i < rcl_params->num_nodes; i++) {
    if (strcmp(node_name, rcl_params->node_names[i]) == 0) {
      if (rcldotnet_params_try_get_parameter_from_node_params(&rcl_params->params[i], name, param_value)) {
        return true;
      }
    }
  }

  // Then check if there is a global override.
  for (int i = 0; i < rcl_params->num_nodes; i++) {
    if (strcmp("/**", rcl_params->node_names[i]) == 0) {
      if (rcldotnet_params_try_get_parameter_from_node_params(&rcl_params->params[i], name, param_value)) {
        return true;
      }
    }
  }

  return false;
}
