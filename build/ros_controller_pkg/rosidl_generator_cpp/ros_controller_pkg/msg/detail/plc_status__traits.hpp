// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice

#ifndef ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__TRAITS_HPP_
#define ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_controller_pkg/msg/detail/plc_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros_controller_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const PlcStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_empty
  {
    out << "is_empty: ";
    rosidl_generator_traits::value_to_yaml(msg.is_empty, out);
    out << ", ";
  }

  // member: fence_open
  {
    out << "fence_open: ";
    rosidl_generator_traits::value_to_yaml(msg.fence_open, out);
    out << ", ";
  }

  // member: door_open
  {
    out << "door_open: ";
    rosidl_generator_traits::value_to_yaml(msg.door_open, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlcStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_empty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_empty: ";
    rosidl_generator_traits::value_to_yaml(msg.is_empty, out);
    out << "\n";
  }

  // member: fence_open
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fence_open: ";
    rosidl_generator_traits::value_to_yaml(msg.fence_open, out);
    out << "\n";
  }

  // member: door_open
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "door_open: ";
    rosidl_generator_traits::value_to_yaml(msg.door_open, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlcStatus & msg, bool use_flow_style = false)
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

}  // namespace ros_controller_pkg

namespace rosidl_generator_traits
{

[[deprecated("use ros_controller_pkg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_controller_pkg::msg::PlcStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_controller_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_controller_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_controller_pkg::msg::PlcStatus & msg)
{
  return ros_controller_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_controller_pkg::msg::PlcStatus>()
{
  return "ros_controller_pkg::msg::PlcStatus";
}

template<>
inline const char * name<ros_controller_pkg::msg::PlcStatus>()
{
  return "ros_controller_pkg/msg/PlcStatus";
}

template<>
struct has_fixed_size<ros_controller_pkg::msg::PlcStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_controller_pkg::msg::PlcStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_controller_pkg::msg::PlcStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__TRAITS_HPP_
