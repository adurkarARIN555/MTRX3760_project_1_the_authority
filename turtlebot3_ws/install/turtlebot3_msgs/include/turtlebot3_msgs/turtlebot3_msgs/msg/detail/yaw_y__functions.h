// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtlebot3_msgs:msg/YawY.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__FUNCTIONS_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "turtlebot3_msgs/msg/detail/yaw_y__struct.h"

/// Initialize msg/YawY message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_msgs__msg__YawY
 * )) before or use
 * turtlebot3_msgs__msg__YawY__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__YawY__init(turtlebot3_msgs__msg__YawY * msg);

/// Finalize msg/YawY message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__YawY__fini(turtlebot3_msgs__msg__YawY * msg);

/// Create msg/YawY message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_msgs__msg__YawY__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__msg__YawY *
turtlebot3_msgs__msg__YawY__create();

/// Destroy msg/YawY message.
/**
 * It calls
 * turtlebot3_msgs__msg__YawY__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__YawY__destroy(turtlebot3_msgs__msg__YawY * msg);

/// Check for msg/YawY message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__YawY__are_equal(const turtlebot3_msgs__msg__YawY * lhs, const turtlebot3_msgs__msg__YawY * rhs);

/// Copy a msg/YawY message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__YawY__copy(
  const turtlebot3_msgs__msg__YawY * input,
  turtlebot3_msgs__msg__YawY * output);

/// Initialize array of msg/YawY messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_msgs__msg__YawY__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__YawY__Sequence__init(turtlebot3_msgs__msg__YawY__Sequence * array, size_t size);

/// Finalize array of msg/YawY messages.
/**
 * It calls
 * turtlebot3_msgs__msg__YawY__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__YawY__Sequence__fini(turtlebot3_msgs__msg__YawY__Sequence * array);

/// Create array of msg/YawY messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_msgs__msg__YawY__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__msg__YawY__Sequence *
turtlebot3_msgs__msg__YawY__Sequence__create(size_t size);

/// Destroy array of msg/YawY messages.
/**
 * It calls
 * turtlebot3_msgs__msg__YawY__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__YawY__Sequence__destroy(turtlebot3_msgs__msg__YawY__Sequence * array);

/// Check for msg/YawY message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__YawY__Sequence__are_equal(const turtlebot3_msgs__msg__YawY__Sequence * lhs, const turtlebot3_msgs__msg__YawY__Sequence * rhs);

/// Copy an array of msg/YawY messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__YawY__Sequence__copy(
  const turtlebot3_msgs__msg__YawY__Sequence * input,
  turtlebot3_msgs__msg__YawY__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__YAW_Y__FUNCTIONS_H_
