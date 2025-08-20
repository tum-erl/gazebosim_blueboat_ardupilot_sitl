// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from blueboat_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#ifndef BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_H_
#define BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetTarget in the package blueboat_interfaces.
typedef struct blueboat_interfaces__srv__SetTarget_Request
{
  double x;
  double y;
} blueboat_interfaces__srv__SetTarget_Request;

// Struct for a sequence of blueboat_interfaces__srv__SetTarget_Request.
typedef struct blueboat_interfaces__srv__SetTarget_Request__Sequence
{
  blueboat_interfaces__srv__SetTarget_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} blueboat_interfaces__srv__SetTarget_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetTarget in the package blueboat_interfaces.
typedef struct blueboat_interfaces__srv__SetTarget_Response
{
  bool accepted;
} blueboat_interfaces__srv__SetTarget_Response;

// Struct for a sequence of blueboat_interfaces__srv__SetTarget_Response.
typedef struct blueboat_interfaces__srv__SetTarget_Response__Sequence
{
  blueboat_interfaces__srv__SetTarget_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} blueboat_interfaces__srv__SetTarget_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_H_
