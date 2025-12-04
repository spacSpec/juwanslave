// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice

#ifndef ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__STRUCT_HPP_
#define ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_controller_pkg__msg__PlcStatus __attribute__((deprecated))
#else
# define DEPRECATED__ros_controller_pkg__msg__PlcStatus __declspec(deprecated)
#endif

namespace ros_controller_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlcStatus_
{
  using Type = PlcStatus_<ContainerAllocator>;

  explicit PlcStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_empty = false;
      this->fence_open = false;
      this->door_open = false;
    }
  }

  explicit PlcStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_empty = false;
      this->fence_open = false;
      this->door_open = false;
    }
  }

  // field types and members
  using _is_empty_type =
    bool;
  _is_empty_type is_empty;
  using _fence_open_type =
    bool;
  _fence_open_type fence_open;
  using _door_open_type =
    bool;
  _door_open_type door_open;

  // setters for named parameter idiom
  Type & set__is_empty(
    const bool & _arg)
  {
    this->is_empty = _arg;
    return *this;
  }
  Type & set__fence_open(
    const bool & _arg)
  {
    this->fence_open = _arg;
    return *this;
  }
  Type & set__door_open(
    const bool & _arg)
  {
    this->door_open = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_controller_pkg::msg::PlcStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_controller_pkg::msg::PlcStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_controller_pkg::msg::PlcStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_controller_pkg::msg::PlcStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_controller_pkg__msg__PlcStatus
    std::shared_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_controller_pkg__msg__PlcStatus
    std::shared_ptr<ros_controller_pkg::msg::PlcStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlcStatus_ & other) const
  {
    if (this->is_empty != other.is_empty) {
      return false;
    }
    if (this->fence_open != other.fence_open) {
      return false;
    }
    if (this->door_open != other.door_open) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlcStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlcStatus_

// alias to use template instance with default allocator
using PlcStatus =
  ros_controller_pkg::msg::PlcStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_controller_pkg

#endif  // ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__STRUCT_HPP_
