// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_pkg_srv_test:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_HPP_
#define CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_pkg_srv_test__srv__CustomService_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_pkg_srv_test__srv__CustomService_Request __declspec(deprecated)
#endif

namespace custom_pkg_srv_test
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CustomService_Request_
{
  using Type = CustomService_Request_<ContainerAllocator>;

  explicit CustomService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit CustomService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _brand_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _brand_type brand;
  using _count_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _count_type count;

  // setters for named parameter idiom
  Type & set__brand(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->brand = _arg;
    return *this;
  }
  Type & set__count(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_pkg_srv_test__srv__CustomService_Request
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_pkg_srv_test__srv__CustomService_Request
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomService_Request_ & other) const
  {
    if (this->brand != other.brand) {
      return false;
    }
    if (this->count != other.count) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomService_Request_

// alias to use template instance with default allocator
using CustomService_Request =
  custom_pkg_srv_test::srv::CustomService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_pkg_srv_test


#ifndef _WIN32
# define DEPRECATED__custom_pkg_srv_test__srv__CustomService_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_pkg_srv_test__srv__CustomService_Response __declspec(deprecated)
#endif

namespace custom_pkg_srv_test
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CustomService_Response_
{
  using Type = CustomService_Response_<ContainerAllocator>;

  explicit CustomService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit CustomService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_pkg_srv_test__srv__CustomService_Response
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_pkg_srv_test__srv__CustomService_Response
    std::shared_ptr<custom_pkg_srv_test::srv::CustomService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomService_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomService_Response_

// alias to use template instance with default allocator
using CustomService_Response =
  custom_pkg_srv_test::srv::CustomService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_pkg_srv_test

namespace custom_pkg_srv_test
{

namespace srv
{

struct CustomService
{
  using Request = custom_pkg_srv_test::srv::CustomService_Request;
  using Response = custom_pkg_srv_test::srv::CustomService_Response;
};

}  // namespace srv

}  // namespace custom_pkg_srv_test

#endif  // CUSTOM_PKG_SRV_TEST__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_HPP_
