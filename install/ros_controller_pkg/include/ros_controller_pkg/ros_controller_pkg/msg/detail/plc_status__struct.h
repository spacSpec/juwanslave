// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice

#ifndef ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__STRUCT_H_
#define ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PlcStatus in the package ros_controller_pkg.
typedef struct ros_controller_pkg__msg__PlcStatus
{
  bool is_empty;
  bool fence_open;
} ros_controller_pkg__msg__PlcStatus;

// Struct for a sequence of ros_controller_pkg__msg__PlcStatus.
typedef struct ros_controller_pkg__msg__PlcStatus__Sequence
{
  ros_controller_pkg__msg__PlcStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_controller_pkg__msg__PlcStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__STRUCT_H_
