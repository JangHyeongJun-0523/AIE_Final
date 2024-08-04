// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/TempPose.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/temp_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_msgs__msg__TempPose__init(robot_msgs__msg__TempPose * msg)
{
  if (!msg) {
    return false;
  }
  // move_flag
  // rotate_flag
  // x
  // y
  // theta
  return true;
}

void
robot_msgs__msg__TempPose__fini(robot_msgs__msg__TempPose * msg)
{
  if (!msg) {
    return;
  }
  // move_flag
  // rotate_flag
  // x
  // y
  // theta
}

bool
robot_msgs__msg__TempPose__are_equal(const robot_msgs__msg__TempPose * lhs, const robot_msgs__msg__TempPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // move_flag
  if (lhs->move_flag != rhs->move_flag) {
    return false;
  }
  // rotate_flag
  if (lhs->rotate_flag != rhs->rotate_flag) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  return true;
}

bool
robot_msgs__msg__TempPose__copy(
  const robot_msgs__msg__TempPose * input,
  robot_msgs__msg__TempPose * output)
{
  if (!input || !output) {
    return false;
  }
  // move_flag
  output->move_flag = input->move_flag;
  // rotate_flag
  output->rotate_flag = input->rotate_flag;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  return true;
}

robot_msgs__msg__TempPose *
robot_msgs__msg__TempPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__TempPose * msg = (robot_msgs__msg__TempPose *)allocator.allocate(sizeof(robot_msgs__msg__TempPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__TempPose));
  bool success = robot_msgs__msg__TempPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__TempPose__destroy(robot_msgs__msg__TempPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__TempPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__TempPose__Sequence__init(robot_msgs__msg__TempPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__TempPose * data = NULL;

  if (size) {
    data = (robot_msgs__msg__TempPose *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__TempPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__TempPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__TempPose__fini(&data[i - 1]);
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
robot_msgs__msg__TempPose__Sequence__fini(robot_msgs__msg__TempPose__Sequence * array)
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
      robot_msgs__msg__TempPose__fini(&array->data[i]);
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

robot_msgs__msg__TempPose__Sequence *
robot_msgs__msg__TempPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__TempPose__Sequence * array = (robot_msgs__msg__TempPose__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__TempPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__TempPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__TempPose__Sequence__destroy(robot_msgs__msg__TempPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__TempPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__TempPose__Sequence__are_equal(const robot_msgs__msg__TempPose__Sequence * lhs, const robot_msgs__msg__TempPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__TempPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__TempPose__Sequence__copy(
  const robot_msgs__msg__TempPose__Sequence * input,
  robot_msgs__msg__TempPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__TempPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__TempPose * data =
      (robot_msgs__msg__TempPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__TempPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__TempPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__TempPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
