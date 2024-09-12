#include "obstacle_avoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance()
: Node("obstacle_avoidance"), FINISH_LINE_Y(3.6)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  sensor_data_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "sensor_data", qos, std::bind(&ObstacleAvoidance::sensor_data_callback, this, std::placeholders::_1));

  yaw_y_sub_ = this->create_subscription<turtlebot3_msgs::msg::YawY>(
  "robot_pose_yaw", qos, std::bind(&ObstacleAvoidance::robot_pose_callback, this, std::placeholders::_1));


  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node has been initialized");
}

ObstacleAvoidance::~ObstacleAvoidance()
{
  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node has been terminated");
}

void ObstacleAvoidance::sensor_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  scan_data_[0] = msg->data[0]; // Front distance
  scan_data_[1] = msg->data[1]; // Left distance
  scan_data_[2] = msg->data[2]; // Right distance
}

void ObstacleAvoidance::robot_pose_callback(const turtlebot3_msgs::msg::YawY::SharedPtr msg)
{
  // Retrieve the robot's pose and yaw from the YawY message
  robot_pose_ = msg->yaw; // Assuming you want to use the yaw value for some purpose
  robot_y_ = msg->y;     // Get the y-coordinate from the message

  // Constants for distance thresholds and speeds
  const double DIST_THRESHOLD_FRONT = 0.35;   // Distance threshold for front wall
  const double DIST_THRESHOLD_LEFT = 0.35;    // Distance threshold for left wall
  const double DIST_THRESHOLD_RIGHT = 0.35;   // Distance threshold for right wall
  const double LINEAR_SPEED = 0.2;            // Linear speed
  const double TURN_SPEED = 1.5;              // Turning speed for sharper U-turns
  const double TURN_SPEED_SLOW = 0.6;        // Slower turning speed for wide turns
  const double TURN_SPEED_FAST_RIGHT = 1.5;  // Faster turning speed for tight right turns

  // Initialize linear and angular speeds
  double linear_speed = LINEAR_SPEED;
  double angular_speed = 0.0;

  if (robot_y_ >= FINISH_LINE_Y) 
  {
    linear_speed = 0.0;
    angular_speed = 0.0;
    RCLCPP_INFO(this->get_logger(), "Robot has crossed the finish line.");
  } 
  // Decision making based on sensor data
  else if (scan_data_[1] < DIST_THRESHOLD_LEFT && scan_data_[0] < DIST_THRESHOLD_FRONT) 
  {
    // Inner corner detected: turn right sharply
    angular_speed = -TURN_SPEED_FAST_RIGHT;
    linear_speed = 0.0;
    RCLCPP_INFO(this->get_logger(), "Inner corner detected: turning right sharply.");
  } 
  else if (scan_data_[1] < DIST_THRESHOLD_LEFT) 
  {
    // Wall on the left is too close: turn right
    angular_speed = -TURN_SPEED;
    RCLCPP_INFO(this->get_logger(), "Wall on the left is too close: turning right.");
  } 
  else if (scan_data_[0] < DIST_THRESHOLD_FRONT) 
  {
    // Wall in front is too close: turn left
    angular_speed = TURN_SPEED;
    RCLCPP_INFO(this->get_logger(), "Wall in front is too close: turning left.");
  } 
  else if (scan_data_[2] < DIST_THRESHOLD_RIGHT) 
  {
    // Wall on the right: move forward with slight left turn
    linear_speed = LINEAR_SPEED;
    angular_speed = TURN_SPEED_SLOW;
    RCLCPP_INFO(this->get_logger(), "Wall on the right: moving forward with slight left turn.");
  } 
  else 
  {
    // No significant obstacles: move forward with slight left turn
    linear_speed = LINEAR_SPEED;
    angular_speed = TURN_SPEED_SLOW;
    RCLCPP_INFO(this->get_logger(), "No significant obstacles: moving forward with slight left turn.");
  }

  // Update robot velocity
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = angular_speed;
  cmd_vel_pub_->publish(cmd_vel);
}

