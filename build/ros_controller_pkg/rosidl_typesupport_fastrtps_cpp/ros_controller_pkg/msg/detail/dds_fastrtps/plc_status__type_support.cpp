// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice
#include "ros_controller_pkg/msg/detail/plc_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros_controller_pkg/msg/detail/plc_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: is_empty
  cdr << (ros_message.is_empty ? true : false);
  // Member: fence_open
  cdr << (ros_message.fence_open ? true : false);
  // Member: door_open
  cdr << (ros_message.door_open ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros_controller_pkg::msg::PlcStatus & ros_message)
{
  // Member: is_empty
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_empty = tmp ? true : false;
  }

  // Member: fence_open
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fence_open = tmp ? true : false;
  }

  // Member: door_open
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.door_open = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
get_serialized_size(
  const ros_controller_pkg::msg::PlcStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: is_empty
  {
    size_t item_size = sizeof(ros_message.is_empty);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fence_open
  {
    size_t item_size = sizeof(ros_message.fence_open);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: door_open
  {
    size_t item_size = sizeof(ros_message.door_open);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_controller_pkg
max_serialized_size_PlcStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: is_empty
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fence_open
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: door_open
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros_controller_pkg::msg::PlcStatus;
    is_plain =
      (
      offsetof(DataType, door_open) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PlcStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros_controller_pkg::msg::PlcStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PlcStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros_controller_pkg::msg::PlcStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PlcStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros_controller_pkg::msg::PlcStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PlcStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PlcStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PlcStatus__callbacks = {
  "ros_controller_pkg::msg",
  "PlcStatus",
  _PlcStatus__cdr_serialize,
  _PlcStatus__cdr_deserialize,
  _PlcStatus__get_serialized_size,
  _PlcStatus__max_serialized_size
};

static rosidl_message_type_support_t _PlcStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PlcStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros_controller_pkg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros_controller_pkg
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_controller_pkg::msg::PlcStatus>()
{
  return &ros_controller_pkg::msg::typesupport_fastrtps_cpp::_PlcStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_controller_pkg, msg, PlcStatus)() {
  return &ros_controller_pkg::msg::typesupport_fastrtps_cpp::_PlcStatus__handle;
}

#ifdef __cplusplus
}
#endif
