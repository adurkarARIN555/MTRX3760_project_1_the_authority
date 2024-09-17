// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_msgs:msg/YawY.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/msg/detail/yaw_y__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
turtlebot3_msgs__msg__YawY__init(turtlebot3_msgs__msg__YawY * msg)
{
  if (!msg) {
    return false;
  }
  // yaw
  // y
  return true;
}

void
turtlebot3_msgs__msg__YawY__fini(turtlebot3_msgs__msg__YawY * msg)
{
  if (!msg) {
    return;
  }
  // yaw
  // y
}

bool
turtlebot3_msgs__msg__YawY__are_equal(const turtlebot3_msgs__msg__YawY * lhs, const turtlebot3_msgs__msg__YawY * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__msg__YawY__copy(
  const turtlebot3_msgs__msg__YawY * input,
  turtlebot3_msgs__msg__YawY * output)
{
  if (!input || !output) {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // y
  output->y = input->y;
  return true;
}

turtlebot3_msgs__msg__YawY *
turtlebot3_msgs__msg__YawY__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_msgs__msg__YawY * msg = (turtlebot3_msgs__msg__YawY *)allocator.allocate(sizeof(turtlebot3_msgs__msg__YawY), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__msg__YawY));
  bool success = turtlebot3_msgs__msg__YawY__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__msg__YawY__destroy(turtlebot3_msgs__msg__YawY * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_msgs__msg__YawY__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_msgs__msg__YawY__Sequence__init(turtlebot3_msgs__msg__YawY__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_msgs__msg__YawY * data = NULL;

  if (size) {
    data = (turtlebot3_msgs__msg__YawY *)allocator.zero_allocate(size, sizeof(turtlebot3_msgs__msg__YawY), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__msg__YawY__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__msg__YawY__fini(&data[i - 1]);
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
turtlebot3_msgs__msg__YawY__Sequence__fini(turtlebot3_msgs__msg__YawY__Sequence * array)
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
      turtlebot3_msgs__msg__YawY__fini(&array->data[i]);
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

turtlebot3_msgs__msg__YawY__Sequence *
turtlebot3_msgs__msg__YawY__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_msgs__msg__YawY__Sequence * array = (turtlebot3_msgs__msg__YawY__Sequence *)allocator.allocate(sizeof(turtlebot3_msgs__msg__YawY__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__msg__YawY__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__msg__YawY__Sequence__destroy(turtlebot3_msgs__msg__YawY__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_msgs__msg__YawY__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_msgs__msg__YawY__Sequence__are_equal(const turtlebot3_msgs__msg__YawY__Sequence * lhs, const turtlebot3_msgs__msg__YawY__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__msg__YawY__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__msg__YawY__Sequence__copy(
  const turtlebot3_msgs__msg__YawY__Sequence * input,
  turtlebot3_msgs__msg__YawY__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__msg__YawY);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_msgs__msg__YawY * data =
      (turtlebot3_msgs__msg__YawY *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__msg__YawY__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__msg__YawY__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__msg__YawY__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
