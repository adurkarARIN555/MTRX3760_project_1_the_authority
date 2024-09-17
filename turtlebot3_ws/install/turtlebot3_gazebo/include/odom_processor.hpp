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