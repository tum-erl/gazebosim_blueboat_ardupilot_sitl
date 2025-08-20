// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from blueboat_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "blueboat_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
#include "blueboat_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "blueboat_interfaces/srv/detail/set_target__functions.h"
#include "blueboat_interfaces/srv/detail/set_target__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  blueboat_interfaces__srv__SetTarget_Request__init(message_memory);
}

void blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_fini_function(void * message_memory)
{
  blueboat_interfaces__srv__SetTarget_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(blueboat_interfaces__srv__SetTarget_Request, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(blueboat_interfaces__srv__SetTarget_Request, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_members = {
  "blueboat_interfaces__srv",  // message namespace
  "SetTarget_Request",  // message name
  2,  // number of fields
  sizeof(blueboat_interfaces__srv__SetTarget_Request),
  blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_member_array,  // message members
  blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle = {
  0,
  &blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_blueboat_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget_Request)() {
  if (!blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle.typesupport_identifier) {
    blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &blueboat_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "blueboat_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "blueboat_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "blueboat_interfaces/srv/detail/set_target__functions.h"
// already included above
// #include "blueboat_interfaces/srv/detail/set_target__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  blueboat_interfaces__srv__SetTarget_Response__init(message_memory);
}

void blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_fini_function(void * message_memory)
{
  blueboat_interfaces__srv__SetTarget_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_member_array[1] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(blueboat_interfaces__srv__SetTarget_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_members = {
  "blueboat_interfaces__srv",  // message namespace
  "SetTarget_Response",  // message name
  1,  // number of fields
  sizeof(blueboat_interfaces__srv__SetTarget_Response),
  blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_member_array,  // message members
  blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle = {
  0,
  &blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_blueboat_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget_Response)() {
  if (!blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle.typesupport_identifier) {
    blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &blueboat_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "blueboat_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "blueboat_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_members = {
  "blueboat_interfaces__srv",  // service namespace
  "SetTarget",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle,
  NULL  // response message
  // blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle
};

static rosidl_service_type_support_t blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle = {
  0,
  &blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_blueboat_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget)() {
  if (!blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle.typesupport_identifier) {
    blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, blueboat_interfaces, srv, SetTarget_Response)()->data;
  }

  return &blueboat_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle;
}
