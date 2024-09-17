// ***************************************
// src/odom_processor.cpp
//
// Implementation for OdomProcessor class
// ***************************************

#include "odom_processor.hpp"

OdometryProcessor::OdometryProcessor() : Node("odometry_processor")
{
  // Create a QOS profile with a history depth of 10
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Subscription to the "odom" topic
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
  "odom", qos, std::bind(&OdometryProcessor::OdomCallback, this, std::placeholders::_1));
  
  // Publisher for the robot's yaw angle
  robot_pose_pub = this->create_publisher<std_msgs::msg::Float64>("robot_pose_yaw", qos);

  RCLCPP_INFO(this->get_logger(), "Odometry processor node has been initialised");
}

OdometryProcessor::~OdometryProcessor()
{
  RCLCPP_INFO(this->get_logger(), "Odometry processor node has been terminated");
}

void OdometryProcessor::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Convert quaternion to roll, pitch, and yaw
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // Create and publish the yaw angle
  auto robot_pose_msg = std_msgs::msg::Float64();
  robot_pose_msg.data = yaw;
  robot_pose_pub->publish(robot_pose_msg);
}
