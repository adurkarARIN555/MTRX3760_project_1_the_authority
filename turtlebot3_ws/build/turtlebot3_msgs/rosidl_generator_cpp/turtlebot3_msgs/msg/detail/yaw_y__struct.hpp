// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_msgs:msg/YawY.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__STRUCT_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__msg__YawY __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__msg__YawY __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YawY_
{
  using Type = YawY_<ContainerAllocator>;

  explicit YawY_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->y = 0.0;
    }
  }

  explicit YawY_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::msg::YawY_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::msg::YawY_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::msg::YawY_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::msg::YawY_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__msg__YawY
    std::shared_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__msg__YawY
    std::shared_ptr<turtlebot3_msgs::msg::YawY_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YawY_ & other) const
  {
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const YawY_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YawY_

// alias to use template instance with default allocator
using YawY =
  turtlebot3_msgs::msg::YawY_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__STRUCT_HPP_
