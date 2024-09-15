#include "obstacle_avoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance()
: Node("obstacle_avoidance") {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  sensor_data_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
  "sensor_data", qos, std::bind(&ObstacleAvoidance::sensor_data_callback, this, std::placeholders::_1));

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
  "robot_pose_yaw", qos, std::bind(&ObstacleAvoidance::robot_pose_callback, this, std::placeholders::_1));

  green_percentage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "green_color_percentage", qos, std::bind(&ObstacleAvoidance::green_percentage_callback, this, std::placeholders::_1));
  
  green_goal_position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "green_goal_position", 
    rclcpp::QoS(10), 
    std::bind(&ObstacleAvoidance::green_goal_position_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node has been initialized");
}

void ObstacleAvoidance::green_percentage_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  green_percentage_ = msg->data;
}

void ObstacleAvoidance::green_goal_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() == 2) {
    green_goal_x_ = msg->data[0];
    green_goal_y_ = msg->data[1];
  }
}

ObstacleAvoidance::~ObstacleAvoidance() {
  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node has been terminated");
}

void ObstacleAvoidance::sensor_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  sensor_data_processor_.update_sensor_data(msg);
}

void ObstacleAvoidance::robot_pose_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  robot_yaw_ = msg->data;

  double front_dist = sensor_data_processor_.get_front_distance();
  double left_dist  = sensor_data_processor_.get_left_distance ();
  double right_dist = sensor_data_processor_.get_right_distance();

  auto cmd_vel = velocity_commander_.generate_velocity_command(front_dist, left_dist, right_dist, green_percentage_, green_goal_x_);
  cmd_vel_pub_->publish(cmd_vel);
}
