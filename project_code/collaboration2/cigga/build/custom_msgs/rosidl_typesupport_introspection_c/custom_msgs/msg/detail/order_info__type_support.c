// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_msgs:msg/OrderInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_msgs/msg/detail/order_info__rosidl_typesupport_introspection_c.h"
#include "custom_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_msgs/msg/detail/order_info__functions.h"
#include "custom_msgs/msg/detail/order_info__struct.h"


// Include directives for member types
// Member `brands`
#include "rosidl_runtime_c/string_functions.h"
// Member `counts`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_msgs__msg__OrderInfo__init(message_memory);
}

void custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_fini_function(void * message_memory)
{
  custom_msgs__msg__OrderInfo__fini(message_memory);
}

size_t custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__size_function__OrderInfo__brands(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_const_function__OrderInfo__brands(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_function__OrderInfo__brands(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__fetch_function__OrderInfo__brands(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_const_function__OrderInfo__brands(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__assign_function__OrderInfo__brands(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_function__OrderInfo__brands(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__resize_function__OrderInfo__brands(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__size_function__OrderInfo__counts(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_const_function__OrderInfo__counts(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_function__OrderInfo__counts(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__fetch_function__OrderInfo__counts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_const_function__OrderInfo__counts(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__assign_function__OrderInfo__counts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_function__OrderInfo__counts(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__resize_function__OrderInfo__counts(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_member_array[2] = {
  {
    "brands",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__OrderInfo, brands),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__size_function__OrderInfo__brands,  // size() function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_const_function__OrderInfo__brands,  // get_const(index) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_function__OrderInfo__brands,  // get(index) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__fetch_function__OrderInfo__brands,  // fetch(index, &value) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__assign_function__OrderInfo__brands,  // assign(index, value) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__resize_function__OrderInfo__brands  // resize(index) function pointer
  },
  {
    "counts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__OrderInfo, counts),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__size_function__OrderInfo__counts,  // size() function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_const_function__OrderInfo__counts,  // get_const(index) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__get_function__OrderInfo__counts,  // get(index) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__fetch_function__OrderInfo__counts,  // fetch(index, &value) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__assign_function__OrderInfo__counts,  // assign(index, value) function pointer
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__resize_function__OrderInfo__counts  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_members = {
  "custom_msgs__msg",  // message namespace
  "OrderInfo",  // message name
  2,  // number of fields
  sizeof(custom_msgs__msg__OrderInfo),
  custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_member_array,  // message members
  custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_type_support_handle = {
  0,
  &custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_msgs, msg, OrderInfo)() {
  if (!custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_type_support_handle.typesupport_identifier) {
    custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_msgs__msg__OrderInfo__rosidl_typesupport_introspection_c__OrderInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
