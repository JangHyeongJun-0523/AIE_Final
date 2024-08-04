// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/GoalPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__GOAL_POSE__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__GOAL_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/goal_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_GoalPose_theta
{
public:
  explicit Init_GoalPose_theta(::robot_msgs::msg::GoalPose & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::GoalPose theta(::robot_msgs::msg::GoalPose::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::GoalPose msg_;
};

class Init_GoalPose_y
{
public:
  explicit Init_GoalPose_y(::robot_msgs::msg::GoalPose & msg)
  : msg_(msg)
  {}
  Init_GoalPose_theta y(::robot_msgs::msg::GoalPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GoalPose_theta(msg_);
  }

private:
  ::robot_msgs::msg::GoalPose msg_;
};

class Init_GoalPose_x
{
public:
  explicit Init_GoalPose_x(::robot_msgs::msg::GoalPose & msg)
  : msg_(msg)
  {}
  Init_GoalPose_y x(::robot_msgs::msg::GoalPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GoalPose_y(msg_);
  }

private:
  ::robot_msgs::msg::GoalPose msg_;
};

class Init_GoalPose_rotate_flag
{
public:
  explicit Init_GoalPose_rotate_flag(::robot_msgs::msg::GoalPose & msg)
  : msg_(msg)
  {}
  Init_GoalPose_x rotate_flag(::robot_msgs::msg::GoalPose::_rotate_flag_type arg)
  {
    msg_.rotate_flag = std::move(arg);
    return Init_GoalPose_x(msg_);
  }

private:
  ::robot_msgs::msg::GoalPose msg_;
};

class Init_GoalPose_move_flag
{
public:
  Init_GoalPose_move_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoalPose_rotate_flag move_flag(::robot_msgs::msg::GoalPose::_move_flag_type arg)
  {
    msg_.move_flag = std::move(arg);
    return Init_GoalPose_rotate_flag(msg_);
  }

private:
  ::robot_msgs::msg::GoalPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::GoalPose>()
{
  return robot_msgs::msg::builder::Init_GoalPose_move_flag();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__GOAL_POSE__BUILDER_HPP_
