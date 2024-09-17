// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_msgs:msg/YawY.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__STRUCT_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/YawY in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__msg__YawY
{
  double yaw;
  double y;
} turtlebot3_msgs__msg__YawY;

// Struct for a sequence of turtlebot3_msgs__msg__YawY.
typedef struct turtlebot3_msgs__msg__YawY__Sequence
{
  turtlebot3_msgs__msg__YawY * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__msg__YawY__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__STRUCT_H_
