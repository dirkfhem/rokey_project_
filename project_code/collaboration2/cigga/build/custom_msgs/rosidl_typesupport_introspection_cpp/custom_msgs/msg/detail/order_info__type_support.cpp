// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from custom_msgs:msg/OrderInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_msgs/msg/detail/order_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace custom_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void OrderInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) custom_msgs::msg::OrderInfo(_init);
}

void OrderInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<custom_msgs::msg::OrderInfo *>(message_memory);
  typed_message->~OrderInfo();
}

size_t size_function__OrderInfo__brands(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__OrderInfo__brands(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderInfo__brands(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderInfo__brands(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__OrderInfo__brands(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__OrderInfo__brands(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__OrderInfo__brands(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__OrderInfo__brands(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__OrderInfo__counts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__OrderInfo__counts(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderInfo__counts(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderInfo__counts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__OrderInfo__counts(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__OrderInfo__counts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__OrderInfo__counts(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__OrderInfo__counts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OrderInfo_message_member_array[2] = {
  {
    "brands",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::OrderInfo, brands),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderInfo__brands,  // size() function pointer
    get_const_function__OrderInfo__brands,  // get_const(index) function pointer
    get_function__OrderInfo__brands,  // get(index) function pointer
    fetch_function__OrderInfo__brands,  // fetch(index, &value) function pointer
    assign_function__OrderInfo__brands,  // assign(index, value) function pointer
    resize_function__OrderInfo__brands  // resize(index) function pointer
  },
  {
    "counts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::OrderInfo, counts),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderInfo__counts,  // size() function pointer
    get_const_function__OrderInfo__counts,  // get_const(index) function pointer
    get_function__OrderInfo__counts,  // get(index) function pointer
    fetch_function__OrderInfo__counts,  // fetch(index, &value) function pointer
    assign_function__OrderInfo__counts,  // assign(index, value) function pointer
    resize_function__OrderInfo__counts  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OrderInfo_message_members = {
  "custom_msgs::msg",  // message namespace
  "OrderInfo",  // message name
  2,  // number of fields
  sizeof(custom_msgs::msg::OrderInfo),
  OrderInfo_message_member_array,  // message members
  OrderInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  OrderInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OrderInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OrderInfo_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace custom_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_msgs::msg::OrderInfo>()
{
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::OrderInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_msgs, msg, OrderInfo)() {
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::OrderInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
