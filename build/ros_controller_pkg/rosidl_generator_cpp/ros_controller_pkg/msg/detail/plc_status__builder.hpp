// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice

#ifndef ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__BUILDER_HPP_
#define ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_controller_pkg/msg/detail/plc_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_controller_pkg
{

namespace msg
{

namespace builder
{

class Init_PlcStatus_door_open
{
public:
  explicit Init_PlcStatus_door_open(::ros_controller_pkg::msg::PlcStatus & msg)
  : msg_(msg)
  {}
  ::ros_controller_pkg::msg::PlcStatus door_open(::ros_controller_pkg::msg::PlcStatus::_door_open_type arg)
  {
    msg_.door_open = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_controller_pkg::msg::PlcStatus msg_;
};

class Init_PlcStatus_fence_open
{
public:
  explicit Init_PlcStatus_fence_open(::ros_controller_pkg::msg::PlcStatus & msg)
  : msg_(msg)
  {}
  Init_PlcStatus_door_open fence_open(::ros_controller_pkg::msg::PlcStatus::_fence_open_type arg)
  {
    msg_.fence_open = std::move(arg);
    return Init_PlcStatus_door_open(msg_);
  }

private:
  ::ros_controller_pkg::msg::PlcStatus msg_;
};

class Init_PlcStatus_is_empty
{
public:
  Init_PlcStatus_is_empty()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlcStatus_fence_open is_empty(::ros_controller_pkg::msg::PlcStatus::_is_empty_type arg)
  {
    msg_.is_empty = std::move(arg);
    return Init_PlcStatus_fence_open(msg_);
  }

private:
  ::ros_controller_pkg::msg::PlcStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_controller_pkg::msg::PlcStatus>()
{
  return ros_controller_pkg::msg::builder::Init_PlcStatus_is_empty();
}

}  // namespace ros_controller_pkg

#endif  // ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__BUILDER_HPP_
