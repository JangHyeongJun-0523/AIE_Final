// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:action/MoveToGoal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__STRUCT_H_
#define ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'theta'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_Goal
{
  rosidl_runtime_c__double__Sequence x;
  rosidl_runtime_c__double__Sequence y;
  rosidl_runtime_c__double__Sequence theta;
} robot_msgs__action__MoveToGoal_Goal;

// Struct for a sequence of robot_msgs__action__MoveToGoal_Goal.
typedef struct robot_msgs__action__MoveToGoal_Goal__Sequence
{
  robot_msgs__action__MoveToGoal_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result_status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_Result
{
  rosidl_runtime_c__String result_status;
  double x;
  double y;
  double theta;
} robot_msgs__action__MoveToGoal_Result;

// Struct for a sequence of robot_msgs__action__MoveToGoal_Result.
typedef struct robot_msgs__action__MoveToGoal_Result__Sequence
{
  robot_msgs__action__MoveToGoal_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback_status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_Feedback
{
  rosidl_runtime_c__String feedback_status;
} robot_msgs__action__MoveToGoal_Feedback;

// Struct for a sequence of robot_msgs__action__MoveToGoal_Feedback.
typedef struct robot_msgs__action__MoveToGoal_Feedback__Sequence
{
  robot_msgs__action__MoveToGoal_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robot_msgs/action/detail/move_to_goal__struct.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_msgs__action__MoveToGoal_Goal goal;
} robot_msgs__action__MoveToGoal_SendGoal_Request;

// Struct for a sequence of robot_msgs__action__MoveToGoal_SendGoal_Request.
typedef struct robot_msgs__action__MoveToGoal_SendGoal_Request__Sequence
{
  robot_msgs__action__MoveToGoal_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robot_msgs__action__MoveToGoal_SendGoal_Response;

// Struct for a sequence of robot_msgs__action__MoveToGoal_SendGoal_Response.
typedef struct robot_msgs__action__MoveToGoal_SendGoal_Response__Sequence
{
  robot_msgs__action__MoveToGoal_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robot_msgs__action__MoveToGoal_GetResult_Request;

// Struct for a sequence of robot_msgs__action__MoveToGoal_GetResult_Request.
typedef struct robot_msgs__action__MoveToGoal_GetResult_Request__Sequence
{
  robot_msgs__action__MoveToGoal_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_GetResult_Response
{
  int8_t status;
  robot_msgs__action__MoveToGoal_Result result;
} robot_msgs__action__MoveToGoal_GetResult_Response;

// Struct for a sequence of robot_msgs__action__MoveToGoal_GetResult_Response.
typedef struct robot_msgs__action__MoveToGoal_GetResult_Response__Sequence
{
  robot_msgs__action__MoveToGoal_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"

/// Struct defined in action/MoveToGoal in the package robot_msgs.
typedef struct robot_msgs__action__MoveToGoal_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_msgs__action__MoveToGoal_Feedback feedback;
} robot_msgs__action__MoveToGoal_FeedbackMessage;

// Struct for a sequence of robot_msgs__action__MoveToGoal_FeedbackMessage.
typedef struct robot_msgs__action__MoveToGoal_FeedbackMessage__Sequence
{
  robot_msgs__action__MoveToGoal_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__action__MoveToGoal_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__STRUCT_H_
