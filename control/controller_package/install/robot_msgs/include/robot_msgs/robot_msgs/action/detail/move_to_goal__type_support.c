// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:action/MoveToGoal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/action/detail/move_to_goal__functions.h"
#include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `x`
// Member `y`
// Member `theta`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_Goal__init(message_memory);
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_Goal__fini(message_memory);
}

size_t robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__size_function__MoveToGoal_Goal__x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__fetch_function__MoveToGoal_Goal__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__assign_function__MoveToGoal_Goal__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__resize_function__MoveToGoal_Goal__x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__size_function__MoveToGoal_Goal__y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__fetch_function__MoveToGoal_Goal__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__assign_function__MoveToGoal_Goal__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__resize_function__MoveToGoal_Goal__y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__size_function__MoveToGoal_Goal__theta(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__theta(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__theta(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__fetch_function__MoveToGoal_Goal__theta(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__theta(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__assign_function__MoveToGoal_Goal__theta(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__theta(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__resize_function__MoveToGoal_Goal__theta(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Goal, x),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__size_function__MoveToGoal_Goal__x,  // size() function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__x,  // get_const(index) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__x,  // get(index) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__fetch_function__MoveToGoal_Goal__x,  // fetch(index, &value) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__assign_function__MoveToGoal_Goal__x,  // assign(index, value) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__resize_function__MoveToGoal_Goal__x  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Goal, y),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__size_function__MoveToGoal_Goal__y,  // size() function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__y,  // get_const(index) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__y,  // get(index) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__fetch_function__MoveToGoal_Goal__y,  // fetch(index, &value) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__assign_function__MoveToGoal_Goal__y,  // assign(index, value) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__resize_function__MoveToGoal_Goal__y  // resize(index) function pointer
  },
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Goal, theta),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__size_function__MoveToGoal_Goal__theta,  // size() function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_const_function__MoveToGoal_Goal__theta,  // get_const(index) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__get_function__MoveToGoal_Goal__theta,  // get(index) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__fetch_function__MoveToGoal_Goal__theta,  // fetch(index, &value) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__assign_function__MoveToGoal_Goal__theta,  // assign(index, value) function pointer
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__resize_function__MoveToGoal_Goal__theta  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_Goal",  // message name
  3,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_Goal),
  robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_Goal)() {
  if (!robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_Goal__rosidl_typesupport_introspection_c__MoveToGoal_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `result_status`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_Result__init(message_memory);
}

void robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_member_array[4] = {
  {
    "result_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Result, result_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Result, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Result, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Result, theta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_Result",  // message name
  4,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_Result),
  robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_Result)() {
  if (!robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_Result__rosidl_typesupport_introspection_c__MoveToGoal_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `feedback_status`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_Feedback__init(message_memory);
}

void robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_member_array[1] = {
  {
    "feedback_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_Feedback, feedback_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_Feedback",  // message name
  1,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_Feedback),
  robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_Feedback)() {
  if (!robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_Feedback__rosidl_typesupport_introspection_c__MoveToGoal_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "robot_msgs/action/move_to_goal.h"
// Member `goal`
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_SendGoal_Request__init(message_memory);
}

void robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_SendGoal_Request),
  robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal_Request)() {
  robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_Goal)();
  if (!robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_SendGoal_Response__init(message_memory);
}

void robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_SendGoal_Response),
  robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal_Response)() {
  robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_members = {
  "robot_msgs__action",  // service namespace
  "MoveToGoal_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_type_support_handle = {
  0,
  &robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal)() {
  if (!robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_type_support_handle.typesupport_identifier) {
    robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_SendGoal_Response)()->data;
  }

  return &robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_GetResult_Request__init(message_memory);
}

void robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_GetResult_Request),
  robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult_Request)() {
  robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_GetResult_Request__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "robot_msgs/action/move_to_goal.h"
// Member `result`
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_GetResult_Response__init(message_memory);
}

void robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_GetResult_Response),
  robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult_Response)() {
  robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_Result)();
  if (!robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_GetResult_Response__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_members = {
  "robot_msgs__action",  // service namespace
  "MoveToGoal_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_type_support_handle = {
  0,
  &robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult)() {
  if (!robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_type_support_handle.typesupport_identifier) {
    robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_GetResult_Response)()->data;
  }

  return &robot_msgs__action__detail__move_to_goal__rosidl_typesupport_introspection_c__MoveToGoal_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__functions.h"
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "robot_msgs/action/move_to_goal.h"
// Member `feedback`
// already included above
// #include "robot_msgs/action/detail/move_to_goal__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__action__MoveToGoal_FeedbackMessage__init(message_memory);
}

void robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_fini_function(void * message_memory)
{
  robot_msgs__action__MoveToGoal_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__action__MoveToGoal_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_members = {
  "robot_msgs__action",  // message namespace
  "MoveToGoal_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(robot_msgs__action__MoveToGoal_FeedbackMessage),
  robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_member_array,  // message members
  robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_type_support_handle = {
  0,
  &robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_FeedbackMessage)() {
  robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, action, MoveToGoal_Feedback)();
  if (!robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__action__MoveToGoal_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToGoal_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
