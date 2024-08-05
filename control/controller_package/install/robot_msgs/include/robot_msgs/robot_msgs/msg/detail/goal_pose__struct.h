// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/GoalPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__GOAL_POSE__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__GOAL_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GoalPose in the package robot_msgs.
typedef struct robot_msgs__msg__GoalPose
{
  rosidl_runtime_c__String robot_id;
  double x;
  double y;
  double theta;
} robot_msgs__msg__GoalPose;

// Struct for a sequence of robot_msgs__msg__GoalPose.
typedef struct robot_msgs__msg__GoalPose__Sequence
{
  robot_msgs__msg__GoalPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__GoalPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__GOAL_POSE__STRUCT_H_
