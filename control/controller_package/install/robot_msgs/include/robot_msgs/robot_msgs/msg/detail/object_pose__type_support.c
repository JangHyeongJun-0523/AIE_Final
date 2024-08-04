// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:msg/ObjectPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/msg/detail/object_pose__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/msg/detail/object_pose__functions.h"
#include "robot_msgs/msg/detail/object_pose__struct.h"


// Include directives for member types
// Member `labels`
#include "rosidl_runtime_c/string_functions.h"
// Member `ranges`
// Member `angles`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__msg__ObjectPose__init(message_memory);
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_fini_function(void * message_memory)
{
  robot_msgs__msg__ObjectPose__fini(message_memory);
}

size_t robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__size_function__ObjectPose__labels(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__labels(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__labels(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__fetch_function__ObjectPose__labels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__labels(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__assign_function__ObjectPose__labels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__labels(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__resize_function__ObjectPose__labels(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__size_function__ObjectPose__ranges(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__ranges(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__ranges(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__fetch_function__ObjectPose__ranges(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__ranges(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__assign_function__ObjectPose__ranges(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__ranges(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__resize_function__ObjectPose__ranges(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__size_function__ObjectPose__angles(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__angles(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__angles(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__fetch_function__ObjectPose__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__angles(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__assign_function__ObjectPose__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__angles(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__resize_function__ObjectPose__angles(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_member_array[5] = {
  {
    "detector_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ObjectPose, detector_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ObjectPose, detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "labels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ObjectPose, labels),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__size_function__ObjectPose__labels,  // size() function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__labels,  // get_const(index) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__labels,  // get(index) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__fetch_function__ObjectPose__labels,  // fetch(index, &value) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__assign_function__ObjectPose__labels,  // assign(index, value) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__resize_function__ObjectPose__labels  // resize(index) function pointer
  },
  {
    "ranges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ObjectPose, ranges),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__size_function__ObjectPose__ranges,  // size() function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__ranges,  // get_const(index) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__ranges,  // get(index) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__fetch_function__ObjectPose__ranges,  // fetch(index, &value) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__assign_function__ObjectPose__ranges,  // assign(index, value) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__resize_function__ObjectPose__ranges  // resize(index) function pointer
  },
  {
    "angles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ObjectPose, angles),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__size_function__ObjectPose__angles,  // size() function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_const_function__ObjectPose__angles,  // get_const(index) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__get_function__ObjectPose__angles,  // get(index) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__fetch_function__ObjectPose__angles,  // fetch(index, &value) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__assign_function__ObjectPose__angles,  // assign(index, value) function pointer
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__resize_function__ObjectPose__angles  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_members = {
  "robot_msgs__msg",  // message namespace
  "ObjectPose",  // message name
  5,  // number of fields
  sizeof(robot_msgs__msg__ObjectPose),
  robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_member_array,  // message members
  robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_type_support_handle = {
  0,
  &robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, msg, ObjectPose)() {
  if (!robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_type_support_handle.typesupport_identifier) {
    robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__msg__ObjectPose__rosidl_typesupport_introspection_c__ObjectPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
