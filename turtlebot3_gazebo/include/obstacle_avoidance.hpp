#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include "odom_processor.hpp"

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance();
  ~ObstacleAvoidance();

private:
  void sensor_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void robot_pose_callback(const turtlebot3_msgs::msg::YawY::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_data_sub_;
  rclcpp::Subscription<turtlebot3_msgs::msg::YawY>::SharedPtr yaw_y_sub_; 
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  double scan_data_[3];
  double robot_pose_;
  double FINISH_LINE_Y;
  double robot_y_;
  double yaw_;
};

#endif  // OBSTACLE_AVOIDANCE_HPP_


