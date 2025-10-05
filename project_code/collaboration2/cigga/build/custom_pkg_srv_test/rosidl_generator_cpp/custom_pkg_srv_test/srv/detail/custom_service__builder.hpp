// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_pkg_srv_test:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__BUILDER_HPP_
#define CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_pkg_srv_test/srv/detail/custom_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_pkg_srv_test
{

namespace srv
{

namespace builder
{

class Init_CustomService_Request_count
{
public:
  explicit Init_CustomService_Request_count(::custom_pkg_srv_test::srv::CustomService_Request & msg)
  : msg_(msg)
  {}
  ::custom_pkg_srv_test::srv::CustomService_Request count(::custom_pkg_srv_test::srv::CustomService_Request::_count_type arg)
  {
    msg_.count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_pkg_srv_test::srv::CustomService_Request msg_;
};

class Init_CustomService_Request_brand
{
public:
  Init_CustomService_Request_brand()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomService_Request_count brand(::custom_pkg_srv_test::srv::CustomService_Request::_brand_type arg)
  {
    msg_.brand = std::move(arg);
    return Init_CustomService_Request_count(msg_);
  }

private:
  ::custom_pkg_srv_test::srv::CustomService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_pkg_srv_test::srv::CustomService_Request>()
{
  return custom_pkg_srv_test::srv::builder::Init_CustomService_Request_brand();
}

}  // namespace custom_pkg_srv_test


namespace custom_pkg_srv_test
{

namespace srv
{

namespace builder
{

class Init_CustomService_Response_success
{
public:
  Init_CustomService_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_pkg_srv_test::srv::CustomService_Response success(::custom_pkg_srv_test::srv::CustomService_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_pkg_srv_test::srv::CustomService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_pkg_srv_test::srv::CustomService_Response>()
{
  return custom_pkg_srv_test::srv::builder::Init_CustomService_Response_success();
}

}  // namespace custom_pkg_srv_test

#endif  // CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__BUILDER_HPP_
