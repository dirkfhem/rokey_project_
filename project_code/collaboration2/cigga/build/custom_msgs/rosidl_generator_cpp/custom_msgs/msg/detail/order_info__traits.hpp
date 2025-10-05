// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/OrderInfo.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/order_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrderInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: brands
  {
    if (msg.brands.size() == 0) {
      out << "brands: []";
    } else {
      out << "brands: [";
      size_t pending_items = msg.brands.size();
      for (auto item : msg.brands) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: counts
  {
    if (msg.counts.size() == 0) {
      out << "counts: []";
    } else {
      out << "counts: [";
      size_t pending_items = msg.counts.size();
      for (auto item : msg.counts) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrderInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: brands
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.brands.size() == 0) {
      out << "brands: []\n";
    } else {
      out << "brands:\n";
      for (auto item : msg.brands) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: counts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.counts.size() == 0) {
      out << "counts: []\n";
    } else {
      out << "counts:\n";
      for (auto item : msg.counts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrderInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::OrderInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::OrderInfo & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::OrderInfo>()
{
  return "custom_msgs::msg::OrderInfo";
}

template<>
inline const char * name<custom_msgs::msg::OrderInfo>()
{
  return "custom_msgs/msg/OrderInfo";
}

template<>
struct has_fixed_size<custom_msgs::msg::OrderInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::OrderInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::OrderInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__ORDER_INFO__TRAITS_HPP_
