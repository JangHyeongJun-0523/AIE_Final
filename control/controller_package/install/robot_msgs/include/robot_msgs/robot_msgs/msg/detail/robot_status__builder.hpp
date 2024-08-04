// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_status
{
public:
  explicit Init_RobotStatus_status(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::RobotStatus status(::robot_msgs::msg::RobotStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_robot_id
{
public:
  Init_RobotStatus_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_status robot_id(::robot_msgs::msg::RobotStatus::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotStatus_status(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::RobotStatus>()
{
  return robot_msgs::msg::builder::Init_RobotStatus_robot_id();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
