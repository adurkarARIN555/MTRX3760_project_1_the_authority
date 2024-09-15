#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_data_processor.hpp"
#include "robot_pose_processor.hpp"
#include "velocity_commander.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/point.hpp"


class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance();
  ~ObstacleAvoidance();

private:
  void sensor_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void robot_pose_callback(const turtlebot3_msgs::msg::YawY::SharedPtr msg);
  void green_goal_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_data_sub_;
  rclcpp::Subscription<turtlebot3_msgs::msg::YawY>::SharedPtr yaw_y_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr green_percentage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr green_goal_position_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  double green_percentage_;
  double green_goal_x_;
  double green_goal_y_;

  SensorDataProcessor sensor_data_processor_;
  RobotPoseProcessor robot_pose_processor_;
  VelocityCommander velocity_commander_;
};

#endif  // OBSTACLE_AVOIDANCE_HPP_