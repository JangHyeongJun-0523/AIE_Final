// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/ObjectPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__ObjectPose __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__ObjectPose __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectPose_
{
  using Type = ObjectPose_<ContainerAllocator>;

  explicit ObjectPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detector_id = 0;
      this->detected = false;
    }
  }

  explicit ObjectPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detector_id = 0;
      this->detected = false;
    }
  }

  // field types and members
  using _detector_id_type =
    int8_t;
  _detector_id_type detector_id;
  using _detected_type =
    bool;
  _detected_type detected;
  using _labels_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _labels_type labels;
  using _ranges_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _ranges_type ranges;
  using _angles_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _angles_type angles;

  // setters for named parameter idiom
  Type & set__detector_id(
    const int8_t & _arg)
  {
    this->detector_id = _arg;
    return *this;
  }
  Type & set__detected(
    const bool & _arg)
  {
    this->detected = _arg;
    return *this;
  }
  Type & set__labels(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->labels = _arg;
    return *this;
  }
  Type & set__ranges(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->ranges = _arg;
    return *this;
  }
  Type & set__angles(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->angles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::ObjectPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::ObjectPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::ObjectPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::ObjectPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__ObjectPose
    std::shared_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__ObjectPose
    std::shared_ptr<robot_msgs::msg::ObjectPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPose_ & other) const
  {
    if (this->detector_id != other.detector_id) {
      return false;
    }
    if (this->detected != other.detected) {
      return false;
    }
    if (this->labels != other.labels) {
      return false;
    }
    if (this->ranges != other.ranges) {
      return false;
    }
    if (this->angles != other.angles) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPose_

// alias to use template instance with default allocator
using ObjectPose =
  robot_msgs::msg::ObjectPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__OBJECT_POSE__STRUCT_HPP_
