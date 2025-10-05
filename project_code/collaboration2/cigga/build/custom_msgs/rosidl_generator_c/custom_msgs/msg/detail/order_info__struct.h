// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/OrderInfo.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'brands'
#include "rosidl_runtime_c/string.h"
// Member 'counts'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/OrderInfo in the package custom_msgs.
typedef struct custom_msgs__msg__OrderInfo
{
  rosidl_runtime_c__String__Sequence brands;
  rosidl_runtime_c__int32__Sequence counts;
} custom_msgs__msg__OrderInfo;

// Struct for a sequence of custom_msgs__msg__OrderInfo.
typedef struct custom_msgs__msg__OrderInfo__Sequence
{
  custom_msgs__msg__OrderInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__OrderInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__STRUCT_H_
