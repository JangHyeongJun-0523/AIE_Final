// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/ObjectPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/object_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectPose_angles
{
public:
  explicit Init_ObjectPose_angles(::robot_msgs::msg::ObjectPose & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::ObjectPose angles(::robot_msgs::msg::ObjectPose::_angles_type arg)
  {
    msg_.angles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::ObjectPose msg_;
};

class Init_ObjectPose_ranges
{
public:
  explicit Init_ObjectPose_ranges(::robot_msgs::msg::ObjectPose & msg)
  : msg_(msg)
  {}
  Init_ObjectPose_angles ranges(::robot_msgs::msg::ObjectPose::_ranges_type arg)
  {
    msg_.ranges = std::move(arg);
    return Init_ObjectPose_angles(msg_);
  }

private:
  ::robot_msgs::msg::ObjectPose msg_;
};

class Init_ObjectPose_labels
{
public:
  explicit Init_ObjectPose_labels(::robot_msgs::msg::ObjectPose & msg)
  : msg_(msg)
  {}
  Init_ObjectPose_ranges labels(::robot_msgs::msg::ObjectPose::_labels_type arg)
  {
    msg_.labels = std::move(arg);
    return Init_ObjectPose_ranges(msg_);
  }

private:
  ::robot_msgs::msg::ObjectPose msg_;
};

class Init_ObjectPose_detected
{
public:
  explicit Init_ObjectPose_detected(::robot_msgs::msg::ObjectPose & msg)
  : msg_(msg)
  {}
  Init_ObjectPose_labels detected(::robot_msgs::msg::ObjectPose::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_ObjectPose_labels(msg_);
  }

private:
  ::robot_msgs::msg::ObjectPose msg_;
};

class Init_ObjectPose_detector_id
{
public:
  Init_ObjectPose_detector_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectPose_detected detector_id(::robot_msgs::msg::ObjectPose::_detector_id_type arg)
  {
    msg_.detector_id = std::move(arg);
    return Init_ObjectPose_detected(msg_);
  }

private:
  ::robot_msgs::msg::ObjectPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::ObjectPose>()
{
  return robot_msgs::msg::builder::Init_ObjectPose_detector_id();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__BUILDER_HPP_
