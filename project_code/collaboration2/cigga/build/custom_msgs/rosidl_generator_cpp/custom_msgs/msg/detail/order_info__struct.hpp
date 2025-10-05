// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/OrderInfo.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__OrderInfo __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__OrderInfo __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrderInfo_
{
  using Type = OrderInfo_<ContainerAllocator>;

  explicit OrderInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit OrderInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _brands_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _brands_type brands;
  using _counts_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _counts_type counts;

  // setters for named parameter idiom
  Type & set__brands(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->brands = _arg;
    return *this;
  }
  Type & set__counts(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->counts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::OrderInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::OrderInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::OrderInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::OrderInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__OrderInfo
    std::shared_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__OrderInfo
    std::shared_ptr<custom_msgs::msg::OrderInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrderInfo_ & other) const
  {
    if (this->brands != other.brands) {
      return false;
    }
    if (this->counts != other.counts) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrderInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrderInfo_

// alias to use template instance with default allocator
using OrderInfo =
  custom_msgs::msg::OrderInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__STRUCT_HPP_
