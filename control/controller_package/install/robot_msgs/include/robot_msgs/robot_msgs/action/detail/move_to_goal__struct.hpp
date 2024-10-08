// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:action/MoveToGoal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__STRUCT_HPP_
#define ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_Goal __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_Goal __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_Goal_
{
  using Type = MoveToGoal_Goal_<ContainerAllocator>;

  explicit MoveToGoal_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MoveToGoal_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_type x;
  using _y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_type y;
  using _theta_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _theta_type theta;

  // setters for named parameter idiom
  Type & set__x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_Goal
    std::shared_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_Goal
    std::shared_ptr<robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_Goal_ & other) const
  {
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
  bool operator!=(const MoveToGoal_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_Goal_

// alias to use template instance with default allocator
using MoveToGoal_Goal =
  robot_msgs::action::MoveToGoal_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs


#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_Result __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_Result __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_Result_
{
  using Type = MoveToGoal_Result_<ContainerAllocator>;

  explicit MoveToGoal_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result_status = "";
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  explicit MoveToGoal_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result_status = "";
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  // field types and members
  using _result_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_status_type result_status;
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
  Type & set__result_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result_status = _arg;
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
    robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_Result
    std::shared_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_Result
    std::shared_ptr<robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_Result_ & other) const
  {
    if (this->result_status != other.result_status) {
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
  bool operator!=(const MoveToGoal_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_Result_

// alias to use template instance with default allocator
using MoveToGoal_Result =
  robot_msgs::action::MoveToGoal_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs


#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_Feedback __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_Feedback_
{
  using Type = MoveToGoal_Feedback_<ContainerAllocator>;

  explicit MoveToGoal_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback_status = "";
    }
  }

  explicit MoveToGoal_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback_status = "";
    }
  }

  // field types and members
  using _feedback_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _feedback_status_type feedback_status;

  // setters for named parameter idiom
  Type & set__feedback_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->feedback_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_Feedback
    std::shared_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_Feedback
    std::shared_ptr<robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_Feedback_ & other) const
  {
    if (this->feedback_status != other.feedback_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToGoal_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_Feedback_

// alias to use template instance with default allocator
using MoveToGoal_Feedback =
  robot_msgs::action::MoveToGoal_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "robot_msgs/action/detail/move_to_goal__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Request __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_SendGoal_Request_
{
  using Type = MoveToGoal_SendGoal_Request_<ContainerAllocator>;

  explicit MoveToGoal_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit MoveToGoal_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const robot_msgs::action::MoveToGoal_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Request
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Request
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToGoal_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_SendGoal_Request_

// alias to use template instance with default allocator
using MoveToGoal_SendGoal_Request =
  robot_msgs::action::MoveToGoal_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Response __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_SendGoal_Response_
{
  using Type = MoveToGoal_SendGoal_Response_<ContainerAllocator>;

  explicit MoveToGoal_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit MoveToGoal_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Response
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_SendGoal_Response
    std::shared_ptr<robot_msgs::action::MoveToGoal_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToGoal_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_SendGoal_Response_

// alias to use template instance with default allocator
using MoveToGoal_SendGoal_Response =
  robot_msgs::action::MoveToGoal_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs

namespace robot_msgs
{

namespace action
{

struct MoveToGoal_SendGoal
{
  using Request = robot_msgs::action::MoveToGoal_SendGoal_Request;
  using Response = robot_msgs::action::MoveToGoal_SendGoal_Response;
};

}  // namespace action

}  // namespace robot_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Request __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_GetResult_Request_
{
  using Type = MoveToGoal_GetResult_Request_<ContainerAllocator>;

  explicit MoveToGoal_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit MoveToGoal_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Request
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Request
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToGoal_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_GetResult_Request_

// alias to use template instance with default allocator
using MoveToGoal_GetResult_Request =
  robot_msgs::action::MoveToGoal_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Response __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_GetResult_Response_
{
  using Type = MoveToGoal_GetResult_Response_<ContainerAllocator>;

  explicit MoveToGoal_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MoveToGoal_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    robot_msgs::action::MoveToGoal_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const robot_msgs::action::MoveToGoal_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Response
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_GetResult_Response
    std::shared_ptr<robot_msgs::action::MoveToGoal_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToGoal_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_GetResult_Response_

// alias to use template instance with default allocator
using MoveToGoal_GetResult_Response =
  robot_msgs::action::MoveToGoal_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs

namespace robot_msgs
{

namespace action
{

struct MoveToGoal_GetResult
{
  using Request = robot_msgs::action::MoveToGoal_GetResult_Request;
  using Response = robot_msgs::action::MoveToGoal_GetResult_Response;
};

}  // namespace action

}  // namespace robot_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "robot_msgs/action/detail/move_to_goal__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__action__MoveToGoal_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__action__MoveToGoal_FeedbackMessage __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToGoal_FeedbackMessage_
{
  using Type = MoveToGoal_FeedbackMessage_<ContainerAllocator>;

  explicit MoveToGoal_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit MoveToGoal_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const robot_msgs::action::MoveToGoal_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_FeedbackMessage
    std::shared_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__action__MoveToGoal_FeedbackMessage
    std::shared_ptr<robot_msgs::action::MoveToGoal_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToGoal_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToGoal_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToGoal_FeedbackMessage_

// alias to use template instance with default allocator
using MoveToGoal_FeedbackMessage =
  robot_msgs::action::MoveToGoal_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace robot_msgs
{

namespace action
{

struct MoveToGoal
{
  /// The goal message defined in the action definition.
  using Goal = robot_msgs::action::MoveToGoal_Goal;
  /// The result message defined in the action definition.
  using Result = robot_msgs::action::MoveToGoal_Result;
  /// The feedback message defined in the action definition.
  using Feedback = robot_msgs::action::MoveToGoal_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = robot_msgs::action::MoveToGoal_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = robot_msgs::action::MoveToGoal_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = robot_msgs::action::MoveToGoal_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct MoveToGoal MoveToGoal;

}  // namespace action

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__ACTION__DETAIL__MOVE_TO_GOAL__STRUCT_HPP_
