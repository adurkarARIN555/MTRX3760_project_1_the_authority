// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:msg/YawY.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__TRAITS_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_msgs/msg/detail/yaw_y__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const YawY & msg,
  std::ostream & out)
{
  out << "{";
  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YawY & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YawY & msg, bool use_flow_style = false)
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

}  // namespace turtlebot3_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_msgs::msg::YawY & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_msgs::msg::YawY & msg)
{
  return turtlebot3_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_msgs::msg::YawY>()
{
  return "turtlebot3_msgs::msg::YawY";
}

template<>
inline const char * name<turtlebot3_msgs::msg::YawY>()
{
  return "turtlebot3_msgs/msg/YawY";
}

template<>
struct has_fixed_size<turtlebot3_msgs::msg::YawY>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::msg::YawY>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::msg::YawY>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__TRAITS_HPP_
