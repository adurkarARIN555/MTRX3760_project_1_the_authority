// ***************************************
// src/obstacle_avoidance.cpp
//
// Implementation for ObstacleAvoidance class
// ***************************************

#include "obstacle_avoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance") 
{
  // Create a QOS profile with a history of the last 10 messages
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Subscription to sensor data
  sensor_data_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
  "sensor_data", qos, std::bind(&ObstacleAvoidance::SensorDataCallback, 
  this, std::placeholders::_1));

  // Subscription to robot pose yaw
  yaw_sub = this->create_subscription<std_msgs::msg::Float64>(
  "robot_pose_yaw", qos, std::bind(&ObstacleAvoidance::RobotPoseCallback, 
  this, std::placeholders::_1));

  // Subscription to green color percentage
  green_percentage_sub = this->create_subscription<std_msgs::msg::Float64>(
  "green_color_percentage", qos, std::bind(&ObstacleAvoidance::GreenPercentageCallback, 
  this, std::placeholders::_1));
  
  // Subscription to green goal position
  green_goal_position_sub = 
  this->create_subscription<std_msgs::msg::Float64MultiArray>(
  "green_goal_position", rclcpp::QoS(10), 
  std::bind(&ObstacleAvoidance::GreenGoalPositionCallback, 
  this, std::placeholders::_1));

  // Publisher for velocity commands
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node has been initialised");
}

void ObstacleAvoidance::GreenPercentageCallback(
const std_msgs::msg::Float64::SharedPtr msg) 
{
  // Update the percentage of green detected
  green_percentage = msg->data;
}

void ObstacleAvoidance::GreenGoalPositionCallback(
const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
   // Update the x and y coordinates of the green goal position 
   // If valid data is received
  if (msg->data.size() == 2) 
  {
    green_goal_x = msg->data[0];
    green_goal_y = msg->data[1];
  }
}

ObstacleAvoidance::~ObstacleAvoidance() 
{
  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node has been terminated");
}

void ObstacleAvoidance::SensorDataCallback(
const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
  // Update sensor data for processing
  sensor_data_processor.UpdateSensorData(msg);
}

void ObstacleAvoidance::RobotPoseCallback(const std_msgs::msg::Float64::SharedPtr msg) 
{
  // Update robot's yaw angle
  robot_pose_processor.UpdatePose(msg);
  robot_yaw = robot_pose_processor.GetYaw();

  // Retrieve sensor distances
  double front_dist = sensor_data_processor.GetFrontDistance();
  double left_dist  = sensor_data_processor.GetLeftDistance ();
  double right_dist = sensor_data_processor.GetRightDistance();

  // Generate velocity command based on sensor data and goals then publish
  auto cmd_vel = velocity_commander.GenerateVelocityCommand(front_dist, 
  left_dist, right_dist, green_percentage, green_goal_x);
  cmd_vel_pub->publish(cmd_vel);
}

// ********************************************************************
// *                          Main Function                           *
// ********************************************************************
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();

  return 0;
}