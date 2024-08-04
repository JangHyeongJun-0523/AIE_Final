// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/ObjectPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'labels'
#include "rosidl_runtime_c/string.h"
// Member 'ranges'
// Member 'angles'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ObjectPose in the package robot_msgs.
typedef struct robot_msgs__msg__ObjectPose
{
  int8_t detector_id;
  bool detected;
  rosidl_runtime_c__String__Sequence labels;
  rosidl_runtime_c__float__Sequence ranges;
  rosidl_runtime_c__float__Sequence angles;
} robot_msgs__msg__ObjectPose;

// Struct for a sequence of robot_msgs__msg__ObjectPose.
typedef struct robot_msgs__msg__ObjectPose__Sequence
{
  robot_msgs__msg__ObjectPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__ObjectPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__STRUCT_H_
