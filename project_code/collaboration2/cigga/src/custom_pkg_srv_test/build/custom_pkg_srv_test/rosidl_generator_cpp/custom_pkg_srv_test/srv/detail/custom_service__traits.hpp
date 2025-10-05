// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_pkg_srv_test:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__TRAITS_HPP_
#define CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_pkg_srv_test/srv/detail/custom_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_pkg_srv_test
{

namespace srv
{

inline void to_flow_style_yaml(
  const CustomService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: brand
  {
    out << "brand: ";
    rosidl_generator_traits::value_to_yaml(msg.brand, out);
    out << ", ";
  }

  // member: count
  {
    out << "count: ";
    rosidl_generator_traits::value_to_yaml(msg.count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CustomService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: brand
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brand: ";
    rosidl_generator_traits::value_to_yaml(msg.brand, out);
    out << "\n";
  }

  // member: count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "count: ";
    rosidl_generator_traits::value_to_yaml(msg.count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CustomService_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_pkg_srv_test

namespace rosidl_generator_traits
{

[[deprecated("use custom_pkg_srv_test::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_pkg_srv_test::srv::CustomService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_pkg_srv_test::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_pkg_srv_test::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_pkg_srv_test::srv::CustomService_Request & msg)
{
  return custom_pkg_srv_test::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_pkg_srv_test::srv::CustomService_Request>()
{
  return "custom_pkg_srv_test::srv::CustomService_Request";
}

template<>
inline const char * name<custom_pkg_srv_test::srv::CustomService_Request>()
{
  return "custom_pkg_srv_test/srv/CustomService_Request";
}

template<>
struct has_fixed_size<custom_pkg_srv_test::srv::CustomService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_pkg_srv_test::srv::CustomService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_pkg_srv_test::srv::CustomService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_pkg_srv_test
{

namespace srv
{

inline void to_flow_style_yaml(
  const CustomService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CustomService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CustomService_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_pkg_srv_test

namespace rosidl_generator_traits
{

[[deprecated("use custom_pkg_srv_test::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_pkg_srv_test::srv::CustomService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_pkg_srv_test::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_pkg_srv_test::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_pkg_srv_test::srv::CustomService_Response & msg)
{
  return custom_pkg_srv_test::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_pkg_srv_test::srv::CustomService_Response>()
{
  return "custom_pkg_srv_test::srv::CustomService_Response";
}

template<>
inline const char * name<custom_pkg_srv_test::srv::CustomService_Response>()
{
  return "custom_pkg_srv_test/srv/CustomService_Response";
}

template<>
struct has_fixed_size<custom_pkg_srv_test::srv::CustomService_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_pkg_srv_test::srv::CustomService_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_pkg_srv_test::srv::CustomService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_pkg_srv_test::srv::CustomService>()
{
  return "custom_pkg_srv_test::srv::CustomService";
}

template<>
inline const char * name<custom_pkg_srv_test::srv::CustomService>()
{
  return "custom_pkg_srv_test/srv/CustomService";
}

template<>
struct has_fixed_size<custom_pkg_srv_test::srv::CustomService>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_pkg_srv_test::srv::CustomService_Request>::value &&
    has_fixed_size<custom_pkg_srv_test::srv::CustomService_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_pkg_srv_test::srv::CustomService>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_pkg_srv_test::srv::CustomService_Request>::value &&
    has_bounded_size<custom_pkg_srv_test::srv::CustomService_Response>::value
  >
{
};

template<>
struct is_service<custom_pkg_srv_test::srv::CustomService>
  : std::true_type
{
};

template<>
struct is_service_request<custom_pkg_srv_test::srv::CustomService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_pkg_srv_test::srv::CustomService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__TRAITS_HPP_
