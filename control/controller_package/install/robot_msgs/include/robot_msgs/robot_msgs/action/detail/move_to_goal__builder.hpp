// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:action/MoveToGoal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__BUILDER_HPP_
#define ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/action/detail/move_to_goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_Goal_theta
{
public:
  explicit Init_MoveToGoal_Goal_theta(::robot_msgs::action::MoveToGoal_Goal & msg)
  : msg_(msg)
  {}
  ::robot_msgs::action::MoveToGoal_Goal theta(::robot_msgs::action::MoveToGoal_Goal::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Goal msg_;
};

class Init_MoveToGoal_Goal_y
{
public:
  explicit Init_MoveToGoal_Goal_y(::robot_msgs::action::MoveToGoal_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToGoal_Goal_theta y(::robot_msgs::action::MoveToGoal_Goal::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MoveToGoal_Goal_theta(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Goal msg_;
};

class Init_MoveToGoal_Goal_x
{
public:
  Init_MoveToGoal_Goal_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToGoal_Goal_y x(::robot_msgs::action::MoveToGoal_Goal::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MoveToGoal_Goal_y(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_Goal>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_Goal_x();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_Result_theta
{
public:
  explicit Init_MoveToGoal_Result_theta(::robot_msgs::action::MoveToGoal_Result & msg)
  : msg_(msg)
  {}
  ::robot_msgs::action::MoveToGoal_Result theta(::robot_msgs::action::MoveToGoal_Result::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Result msg_;
};

class Init_MoveToGoal_Result_y
{
public:
  explicit Init_MoveToGoal_Result_y(::robot_msgs::action::MoveToGoal_Result & msg)
  : msg_(msg)
  {}
  Init_MoveToGoal_Result_theta y(::robot_msgs::action::MoveToGoal_Result::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MoveToGoal_Result_theta(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Result msg_;
};

class Init_MoveToGoal_Result_x
{
public:
  explicit Init_MoveToGoal_Result_x(::robot_msgs::action::MoveToGoal_Result & msg)
  : msg_(msg)
  {}
  Init_MoveToGoal_Result_y x(::robot_msgs::action::MoveToGoal_Result::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MoveToGoal_Result_y(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Result msg_;
};

class Init_MoveToGoal_Result_result_status
{
public:
  Init_MoveToGoal_Result_result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToGoal_Result_x result_status(::robot_msgs::action::MoveToGoal_Result::_result_status_type arg)
  {
    msg_.result_status = std::move(arg);
    return Init_MoveToGoal_Result_x(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_Result>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_Result_result_status();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_Feedback_feedback_status
{
public:
  Init_MoveToGoal_Feedback_feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::action::MoveToGoal_Feedback feedback_status(::robot_msgs::action::MoveToGoal_Feedback::_feedback_status_type arg)
  {
    msg_.feedback_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_Feedback>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_Feedback_feedback_status();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_SendGoal_Request_goal
{
public:
  explicit Init_MoveToGoal_SendGoal_Request_goal(::robot_msgs::action::MoveToGoal_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robot_msgs::action::MoveToGoal_SendGoal_Request goal(::robot_msgs::action::MoveToGoal_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_SendGoal_Request msg_;
};

class Init_MoveToGoal_SendGoal_Request_goal_id
{
public:
  Init_MoveToGoal_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToGoal_SendGoal_Request_goal goal_id(::robot_msgs::action::MoveToGoal_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveToGoal_SendGoal_Request_goal(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_SendGoal_Request>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_SendGoal_Request_goal_id();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_SendGoal_Response_stamp
{
public:
  explicit Init_MoveToGoal_SendGoal_Response_stamp(::robot_msgs::action::MoveToGoal_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::action::MoveToGoal_SendGoal_Response stamp(::robot_msgs::action::MoveToGoal_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_SendGoal_Response msg_;
};

class Init_MoveToGoal_SendGoal_Response_accepted
{
public:
  Init_MoveToGoal_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToGoal_SendGoal_Response_stamp accepted(::robot_msgs::action::MoveToGoal_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveToGoal_SendGoal_Response_stamp(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_SendGoal_Response>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_SendGoal_Response_accepted();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_GetResult_Request_goal_id
{
public:
  Init_MoveToGoal_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::action::MoveToGoal_GetResult_Request goal_id(::robot_msgs::action::MoveToGoal_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_GetResult_Request>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_GetResult_Request_goal_id();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_GetResult_Response_result
{
public:
  explicit Init_MoveToGoal_GetResult_Response_result(::robot_msgs::action::MoveToGoal_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::action::MoveToGoal_GetResult_Response result(::robot_msgs::action::MoveToGoal_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_GetResult_Response msg_;
};

class Init_MoveToGoal_GetResult_Response_status
{
public:
  Init_MoveToGoal_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToGoal_GetResult_Response_result status(::robot_msgs::action::MoveToGoal_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveToGoal_GetResult_Response_result(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_GetResult_Response>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_GetResult_Response_status();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace action
{

namespace builder
{

class Init_MoveToGoal_FeedbackMessage_feedback
{
public:
  explicit Init_MoveToGoal_FeedbackMessage_feedback(::robot_msgs::action::MoveToGoal_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robot_msgs::action::MoveToGoal_FeedbackMessage feedback(::robot_msgs::action::MoveToGoal_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_FeedbackMessage msg_;
};

class Init_MoveToGoal_FeedbackMessage_goal_id
{
public:
  Init_MoveToGoal_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToGoal_FeedbackMessage_feedback goal_id(::robot_msgs::action::MoveToGoal_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveToGoal_FeedbackMessage_feedback(msg_);
  }

private:
  ::robot_msgs::action::MoveToGoal_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::action::MoveToGoal_FeedbackMessage>()
{
  return robot_msgs::action::builder::Init_MoveToGoal_FeedbackMessage_goal_id();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__BUILDER_HPP_
