// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from custom_pkg_srv_test:srv/CustomService.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "custom_pkg_srv_test/srv/detail/custom_service__struct.h"
#include "custom_pkg_srv_test/srv/detail/custom_service__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace custom_pkg_srv_test
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _CustomService_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CustomService_Request_type_support_ids_t;

static const _CustomService_Request_type_support_ids_t _CustomService_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CustomService_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CustomService_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CustomService_Request_type_support_symbol_names_t _CustomService_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_pkg_srv_test, srv, CustomService_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_pkg_srv_test, srv, CustomService_Request)),
  }
};

typedef struct _CustomService_Request_type_support_data_t
{
  void * data[2];
} _CustomService_Request_type_support_data_t;

static _CustomService_Request_type_support_data_t _CustomService_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CustomService_Request_message_typesupport_map = {
  2,
  "custom_pkg_srv_test",
  &_CustomService_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CustomService_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CustomService_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CustomService_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CustomService_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_pkg_srv_test

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_pkg_srv_test, srv, CustomService_Request)() {
  return &::custom_pkg_srv_test::srv::rosidl_typesupport_c::CustomService_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_pkg_srv_test/srv/detail/custom_service__struct.h"
// already included above
// #include "custom_pkg_srv_test/srv/detail/custom_service__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_pkg_srv_test
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _CustomService_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CustomService_Response_type_support_ids_t;

static const _CustomService_Response_type_support_ids_t _CustomService_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CustomService_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CustomService_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CustomService_Response_type_support_symbol_names_t _CustomService_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_pkg_srv_test, srv, CustomService_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_pkg_srv_test, srv, CustomService_Response)),
  }
};

typedef struct _CustomService_Response_type_support_data_t
{
  void * data[2];
} _CustomService_Response_type_support_data_t;

static _CustomService_Response_type_support_data_t _CustomService_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CustomService_Response_message_typesupport_map = {
  2,
  "custom_pkg_srv_test",
  &_CustomService_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CustomService_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CustomService_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CustomService_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CustomService_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_pkg_srv_test

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_pkg_srv_test, srv, CustomService_Response)() {
  return &::custom_pkg_srv_test::srv::rosidl_typesupport_c::CustomService_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_pkg_srv_test/srv/detail/custom_service__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_pkg_srv_test
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _CustomService_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CustomService_type_support_ids_t;

static const _CustomService_type_support_ids_t _CustomService_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CustomService_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CustomService_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CustomService_type_support_symbol_names_t _CustomService_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_pkg_srv_test, srv, CustomService)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_pkg_srv_test, srv, CustomService)),
  }
};

typedef struct _CustomService_type_support_data_t
{
  void * data[2];
} _CustomService_type_support_data_t;

static _CustomService_type_support_data_t _CustomService_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CustomService_service_typesupport_map = {
  2,
  "custom_pkg_srv_test",
  &_CustomService_service_typesupport_ids.typesupport_identifier[0],
  &_CustomService_service_typesupport_symbol_names.symbol_name[0],
  &_CustomService_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CustomService_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CustomService_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_pkg_srv_test

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, custom_pkg_srv_test, srv, CustomService)() {
  return &::custom_pkg_srv_test::srv::rosidl_typesupport_c::CustomService_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
