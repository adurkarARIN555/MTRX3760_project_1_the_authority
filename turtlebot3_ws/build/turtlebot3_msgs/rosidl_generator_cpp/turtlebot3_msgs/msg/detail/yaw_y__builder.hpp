// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:msg/YawY.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__BUILDER_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/msg/detail/yaw_y__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace msg
{

namespace builder
{

class Init_YawY_y
{
public:
  explicit Init_YawY_y(::turtlebot3_msgs::msg::YawY & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::msg::YawY y(::turtlebot3_msgs::msg::YawY::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::msg::YawY msg_;
};

class Init_YawY_yaw
{
public:
  Init_YawY_yaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YawY_y yaw(::turtlebot3_msgs::msg::YawY::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_YawY_y(msg_);
  }

private:
  ::turtlebot3_msgs::msg::YawY msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::msg::YawY>()
{
  return turtlebot3_msgs::msg::builder::Init_YawY_yaw();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__BUILDER_HPP_
