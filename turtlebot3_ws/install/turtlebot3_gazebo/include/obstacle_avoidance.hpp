#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include "sensor_data_processor.hpp"
#include "robot_pose_processor.hpp"
#include "velocity_commander.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ObstacleAvoidance : public rclcpp::Node 
{
public:
   ObstacleAvoidance();
  ~ObstacleAvoidance();

private:
  // Callback functions for handling incoming messages
  void SensorDataCallback       (const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void RobotPoseCallback        (const std_msgs::msg::Float64::SharedPtr msg);
  void GreenPercentageCallback  (const std_msgs::msg::Float64::SharedPtr msg);
  void GreenGoalPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ROS subscribers for different topics
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr green_percentage_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr green_goal_position_sub;
  
  // ROS publisher for commanding the turtlebot's velocity
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  // Variables for storing sensor data and turtlebot state
  double green_percentage;
  double green_goal_x;
  double green_goal_y;
  double robot_yaw;

  // Components for processing sensor data, turtlebot pose, and velocity commands
  SensorDataProcessor sensor_data_processor;
  RobotPoseProcessor  robot_pose_processor;
  VelocityCommander   velocity_commander;
};

#endif
