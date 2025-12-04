// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice

#ifndef ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__FUNCTIONS_H_
#define ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ros_controller_pkg/msg/rosidl_generator_c__visibility_control.h"

#include "ros_controller_pkg/msg/detail/plc_status__struct.h"

/// Initialize msg/PlcStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ros_controller_pkg__msg__PlcStatus
 * )) before or use
 * ros_controller_pkg__msg__PlcStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
bool
ros_controller_pkg__msg__PlcStatus__init(ros_controller_pkg__msg__PlcStatus * msg);

/// Finalize msg/PlcStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
void
ros_controller_pkg__msg__PlcStatus__fini(ros_controller_pkg__msg__PlcStatus * msg);

/// Create msg/PlcStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ros_controller_pkg__msg__PlcStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
ros_controller_pkg__msg__PlcStatus *
ros_controller_pkg__msg__PlcStatus__create();

/// Destroy msg/PlcStatus message.
/**
 * It calls
 * ros_controller_pkg__msg__PlcStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
void
ros_controller_pkg__msg__PlcStatus__destroy(ros_controller_pkg__msg__PlcStatus * msg);

/// Check for msg/PlcStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
bool
ros_controller_pkg__msg__PlcStatus__are_equal(const ros_controller_pkg__msg__PlcStatus * lhs, const ros_controller_pkg__msg__PlcStatus * rhs);

/// Copy a msg/PlcStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
bool
ros_controller_pkg__msg__PlcStatus__copy(
  const ros_controller_pkg__msg__PlcStatus * input,
  ros_controller_pkg__msg__PlcStatus * output);

/// Initialize array of msg/PlcStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * ros_controller_pkg__msg__PlcStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
bool
ros_controller_pkg__msg__PlcStatus__Sequence__init(ros_controller_pkg__msg__PlcStatus__Sequence * array, size_t size);

/// Finalize array of msg/PlcStatus messages.
/**
 * It calls
 * ros_controller_pkg__msg__PlcStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
void
ros_controller_pkg__msg__PlcStatus__Sequence__fini(ros_controller_pkg__msg__PlcStatus__Sequence * array);

/// Create array of msg/PlcStatus messages.
/**
 * It allocates the memory for the array and calls
 * ros_controller_pkg__msg__PlcStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
ros_controller_pkg__msg__PlcStatus__Sequence *
ros_controller_pkg__msg__PlcStatus__Sequence__create(size_t size);

/// Destroy array of msg/PlcStatus messages.
/**
 * It calls
 * ros_controller_pkg__msg__PlcStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
void
ros_controller_pkg__msg__PlcStatus__Sequence__destroy(ros_controller_pkg__msg__PlcStatus__Sequence * array);

/// Check for msg/PlcStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
bool
ros_controller_pkg__msg__PlcStatus__Sequence__are_equal(const ros_controller_pkg__msg__PlcStatus__Sequence * lhs, const ros_controller_pkg__msg__PlcStatus__Sequence * rhs);

/// Copy an array of msg/PlcStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros_controller_pkg
bool
ros_controller_pkg__msg__PlcStatus__Sequence__copy(
  const ros_controller_pkg__msg__PlcStatus__Sequence * input,
  ros_controller_pkg__msg__PlcStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROS_CONTROLLER_PKG__MSG__DETAIL__PLC_STATUS__FUNCTIONS_H_
