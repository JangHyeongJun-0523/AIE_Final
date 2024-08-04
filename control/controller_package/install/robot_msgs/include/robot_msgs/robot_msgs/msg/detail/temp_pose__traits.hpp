// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/TempPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__TEMP_POSE__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__TEMP_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/temp_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TempPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: move_flag
  {
    out << "move_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.move_flag, out);
    out << ", ";
  }

  // member: rotate_flag
  {
    out << "rotate_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.rotate_flag, out);
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
  const TempPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: move_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.move_flag, out);
    out << "\n";
  }

  // member: rotate_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rotate_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.rotate_flag, out);
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

inline std::string to_yaml(const TempPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::msg::TempPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::TempPose & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::TempPose>()
{
  return "robot_msgs::msg::TempPose";
}

template<>
inline const char * name<robot_msgs::msg::TempPose>()
{
  return "robot_msgs/msg/TempPose";
}

template<>
struct has_fixed_size<robot_msgs::msg::TempPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_msgs::msg::TempPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_msgs::msg::TempPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__TEMP_POSE__TRAITS_HPP_
