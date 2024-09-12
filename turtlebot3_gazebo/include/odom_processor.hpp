#ifndef ODOM_PROCESSOR_HPP_
#define ODOM_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include "turtlebot3_msgs/msg/yaw_y.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>


class OdometryProcessor : public rclcpp::Node
{
public:
  OdometryProcessor();
  ~OdometryProcessor();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<turtlebot3_msgs::msg::YawY>::SharedPtr robot_pose_pub_;
};

#endif  // ODOM_PROCESSOR_HPP_
