// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/TempPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__TEMP_POSE__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__TEMP_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__TempPose __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__TempPose __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TempPose_
{
  using Type = TempPose_<ContainerAllocator>;

  explicit TempPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->move_flag = false;
      this->rotate_flag = false;
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  explicit TempPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->move_flag = false;
      this->rotate_flag = false;
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  // field types and members
  using _move_flag_type =
    bool;
  _move_flag_type move_flag;
  using _rotate_flag_type =
    bool;
  _rotate_flag_type rotate_flag;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _theta_type =
    double;
  _theta_type theta;

  // setters for named parameter idiom
  Type & set__move_flag(
    const bool & _arg)
  {
    this->move_flag = _arg;
    return *this;
  }
  Type & set__rotate_flag(
    const bool & _arg)
  {
    this->rotate_flag = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::TempPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::TempPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::TempPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::TempPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::TempPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::TempPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::TempPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::TempPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::TempPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::TempPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__TempPose
    std::shared_ptr<robot_msgs::msg::TempPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__TempPose
    std::shared_ptr<robot_msgs::msg::TempPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TempPose_ & other) const
  {
    if (this->move_flag != other.move_flag) {
      return false;
    }
    if (this->rotate_flag != other.rotate_flag) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const TempPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TempPose_

// alias to use template instance with default allocator
using TempPose =
  robot_msgs::msg::TempPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__TEMP_POSE__STRUCT_HPP_
