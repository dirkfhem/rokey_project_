// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/OrderInfo.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/order_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_OrderInfo_counts
{
public:
  explicit Init_OrderInfo_counts(::custom_msgs::msg::OrderInfo & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::OrderInfo counts(::custom_msgs::msg::OrderInfo::_counts_type arg)
  {
    msg_.counts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::OrderInfo msg_;
};

class Init_OrderInfo_brands
{
public:
  Init_OrderInfo_brands()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderInfo_counts brands(::custom_msgs::msg::OrderInfo::_brands_type arg)
  {
    msg_.brands = std::move(arg);
    return Init_OrderInfo_counts(msg_);
  }

private:
  ::custom_msgs::msg::OrderInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::OrderInfo>()
{
  return custom_msgs::msg::builder::Init_OrderInfo_brands();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__BUILDER_HPP_
