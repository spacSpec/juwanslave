// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice

#ifndef ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_controller_pkg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ros_controller_pkg/msg/detail/plc_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ros_controller_pkg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
cdr_serialize(
  const ros_controller_pkg::msg::PlcStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros_controller_pkg::msg::PlcStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
get_serialized_size(
  const ros_controller_pkg::msg::PlcStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
max_serialized_size_PlcStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros_controller_pkg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_controller_pkg, msg, PlcStatus)();

#ifdef __cplusplus
}
#endif

#endif  // ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
