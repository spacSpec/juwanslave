// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_controller_pkg:msg/PlcStatus.idl
// generated code does not contain a copyright notice
#include "ros_controller_pkg/msg/detail/plc_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros_controller_pkg__msg__PlcStatus__init(ros_controller_pkg__msg__PlcStatus * msg)
{
  if (!msg) {
    return false;
  }
  // is_empty
  // fence_open
  return true;
}

void
ros_controller_pkg__msg__PlcStatus__fini(ros_controller_pkg__msg__PlcStatus * msg)
{
  if (!msg) {
    return;
  }
  // is_empty
  // fence_open
}

bool
ros_controller_pkg__msg__PlcStatus__are_equal(const ros_controller_pkg__msg__PlcStatus * lhs, const ros_controller_pkg__msg__PlcStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_empty
  if (lhs->is_empty != rhs->is_empty) {
    return false;
  }
  // fence_open
  if (lhs->fence_open != rhs->fence_open) {
    return false;
  }
  return true;
}

bool
ros_controller_pkg__msg__PlcStatus__copy(
  const ros_controller_pkg__msg__PlcStatus * input,
  ros_controller_pkg__msg__PlcStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // is_empty
  output->is_empty = input->is_empty;
  // fence_open
  output->fence_open = input->fence_open;
  return true;
}

ros_controller_pkg__msg__PlcStatus *
ros_controller_pkg__msg__PlcStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_controller_pkg__msg__PlcStatus * msg = (ros_controller_pkg__msg__PlcStatus *)allocator.allocate(sizeof(ros_controller_pkg__msg__PlcStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_controller_pkg__msg__PlcStatus));
  bool success = ros_controller_pkg__msg__PlcStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_controller_pkg__msg__PlcStatus__destroy(ros_controller_pkg__msg__PlcStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_controller_pkg__msg__PlcStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_controller_pkg__msg__PlcStatus__Sequence__init(ros_controller_pkg__msg__PlcStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_controller_pkg__msg__PlcStatus * data = NULL;

  if (size) {
    data = (ros_controller_pkg__msg__PlcStatus *)allocator.zero_allocate(size, sizeof(ros_controller_pkg__msg__PlcStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_controller_pkg__msg__PlcStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_controller_pkg__msg__PlcStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros_controller_pkg__msg__PlcStatus__Sequence__fini(ros_controller_pkg__msg__PlcStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros_controller_pkg__msg__PlcStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros_controller_pkg__msg__PlcStatus__Sequence *
ros_controller_pkg__msg__PlcStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_controller_pkg__msg__PlcStatus__Sequence * array = (ros_controller_pkg__msg__PlcStatus__Sequence *)allocator.allocate(sizeof(ros_controller_pkg__msg__PlcStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_controller_pkg__msg__PlcStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_controller_pkg__msg__PlcStatus__Sequence__destroy(ros_controller_pkg__msg__PlcStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_controller_pkg__msg__PlcStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_controller_pkg__msg__PlcStatus__Sequence__are_equal(const ros_controller_pkg__msg__PlcStatus__Sequence * lhs, const ros_controller_pkg__msg__PlcStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_controller_pkg__msg__PlcStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_controller_pkg__msg__PlcStatus__Sequence__copy(
  const ros_controller_pkg__msg__PlcStatus__Sequence * input,
  ros_controller_pkg__msg__PlcStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_controller_pkg__msg__PlcStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_controller_pkg__msg__PlcStatus * data =
      (ros_controller_pkg__msg__PlcStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_controller_pkg__msg__PlcStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_controller_pkg__msg__PlcStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_controller_pkg__msg__PlcStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
