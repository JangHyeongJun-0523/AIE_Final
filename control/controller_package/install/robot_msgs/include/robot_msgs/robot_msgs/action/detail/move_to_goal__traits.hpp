// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:action/MoveToGoal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__TRAITS_HPP_
#define ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/action/detail/move_to_goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    if (msg.x.size() == 0) {
      out << "x: []";
    } else {
      out << "x: [";
      size_t pending_items = msg.x.size();
      for (auto item : msg.x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y
  {
    if (msg.y.size() == 0) {
      out << "y: []";
    } else {
      out << "y: [";
      size_t pending_items = msg.y.size();
      for (auto item : msg.y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: theta
  {
    if (msg.theta.size() == 0) {
      out << "theta: []";
    } else {
      out << "theta: [";
      size_t pending_items = msg.theta.size();
      for (auto item : msg.theta) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x.size() == 0) {
      out << "x: []\n";
    } else {
      out << "x:\n";
      for (auto item : msg.x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y.size() == 0) {
      out << "y: []\n";
    } else {
      out << "y:\n";
      for (auto item : msg.y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.theta.size() == 0) {
      out << "theta: []\n";
    } else {
      out << "theta:\n";
      for (auto item : msg.theta) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_Goal & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_Goal>()
{
  return "robot_msgs::action::MoveToGoal_Goal";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_Goal>()
{
  return "robot_msgs/action/MoveToGoal_Goal";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: result_status
  {
    out << "result_status: ";
    rosidl_generator_traits::value_to_yaml(msg.result_status, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result_status: ";
    rosidl_generator_traits::value_to_yaml(msg.result_status, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_Result & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_Result>()
{
  return "robot_msgs::action::MoveToGoal_Result";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_Result>()
{
  return "robot_msgs/action/MoveToGoal_Result";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: feedback_status
  {
    out << "feedback_status: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: feedback_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback_status: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_Feedback & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_Feedback>()
{
  return "robot_msgs::action::MoveToGoal_Feedback";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_Feedback>()
{
  return "robot_msgs/action/MoveToGoal_Feedback";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "robot_msgs/action/detail/move_to_goal__traits.hpp"

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_SendGoal_Request & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_SendGoal_Request>()
{
  return "robot_msgs::action::MoveToGoal_SendGoal_Request";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_SendGoal_Request>()
{
  return "robot_msgs/action/MoveToGoal_SendGoal_Request";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<robot_msgs::action::MoveToGoal_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<robot_msgs::action::MoveToGoal_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_SendGoal_Response & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_SendGoal_Response>()
{
  return "robot_msgs::action::MoveToGoal_SendGoal_Response";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_SendGoal_Response>()
{
  return "robot_msgs/action/MoveToGoal_SendGoal_Response";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_SendGoal>()
{
  return "robot_msgs::action::MoveToGoal_SendGoal";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_SendGoal>()
{
  return "robot_msgs/action/MoveToGoal_SendGoal";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_msgs::action::MoveToGoal_SendGoal_Request>::value &&
    has_fixed_size<robot_msgs::action::MoveToGoal_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_msgs::action::MoveToGoal_SendGoal_Request>::value &&
    has_bounded_size<robot_msgs::action::MoveToGoal_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<robot_msgs::action::MoveToGoal_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<robot_msgs::action::MoveToGoal_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_msgs::action::MoveToGoal_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_GetResult_Request & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_GetResult_Request>()
{
  return "robot_msgs::action::MoveToGoal_GetResult_Request";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_GetResult_Request>()
{
  return "robot_msgs/action/MoveToGoal_GetResult_Request";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_msgs/action/detail/move_to_goal__traits.hpp"

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_GetResult_Response & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_GetResult_Response>()
{
  return "robot_msgs::action::MoveToGoal_GetResult_Response";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_GetResult_Response>()
{
  return "robot_msgs/action/MoveToGoal_GetResult_Response";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<robot_msgs::action::MoveToGoal_Result>::value> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<robot_msgs::action::MoveToGoal_Result>::value> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_GetResult>()
{
  return "robot_msgs::action::MoveToGoal_GetResult";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_GetResult>()
{
  return "robot_msgs/action/MoveToGoal_GetResult";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_msgs::action::MoveToGoal_GetResult_Request>::value &&
    has_fixed_size<robot_msgs::action::MoveToGoal_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_msgs::action::MoveToGoal_GetResult_Request>::value &&
    has_bounded_size<robot_msgs::action::MoveToGoal_GetResult_Response>::value
  >
{
};

template<>
struct is_service<robot_msgs::action::MoveToGoal_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<robot_msgs::action::MoveToGoal_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_msgs::action::MoveToGoal_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "robot_msgs/action/detail/move_to_goal__traits.hpp"

namespace robot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToGoal_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToGoal_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToGoal_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::action::MoveToGoal_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::action::MoveToGoal_FeedbackMessage & msg)
{
  return robot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::action::MoveToGoal_FeedbackMessage>()
{
  return "robot_msgs::action::MoveToGoal_FeedbackMessage";
}

template<>
inline const char * name<robot_msgs::action::MoveToGoal_FeedbackMessage>()
{
  return "robot_msgs/action/MoveToGoal_FeedbackMessage";
}

template<>
struct has_fixed_size<robot_msgs::action::MoveToGoal_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<robot_msgs::action::MoveToGoal_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<robot_msgs::action::MoveToGoal_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<robot_msgs::action::MoveToGoal_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<robot_msgs::action::MoveToGoal_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<robot_msgs::action::MoveToGoal>
  : std::true_type
{
};

template<>
struct is_action_goal<robot_msgs::action::MoveToGoal_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<robot_msgs::action::MoveToGoal_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<robot_msgs::action::MoveToGoal_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__TRAITS_HPP_
