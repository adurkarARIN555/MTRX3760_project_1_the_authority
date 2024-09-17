// ********************************************************************************
// include/odometry_processor.hpp
//
// This header file defines the OdometryProcessor class, a ROS 2 node responsible for 
// processing odometry data to determine the robot's orientation (roll, pitch, and yaw).
// It subscribes to odometry messages, extracts the orientation, and publishes the yaw
// angle, which can be used for navigation and control purposes.
//
// Author Info: Eashan Garg, Arin Adurkar, Savith Karunanayaka, Joel Pivetta
// Year: 2024
// ********************************************************************************

#ifndef ODOM_PROCESSOR_HPP_
#define ODOM_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdometryProcessor : public rclcpp::Node
{
public:
   OdometryProcessor();
  ~OdometryProcessor();

private:
  // Roll, pitch and yaw angle variables in radians
  double roll;
  double pitch;
  double yaw;

  // Callback function to process odometry data
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Subscription to odometry messages
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  // Publisher for robot pose (yaw) data
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr robot_pose_pub;
};
#endif