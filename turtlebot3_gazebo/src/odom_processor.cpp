#include "odom_processor.hpp"

OdometryProcessor::OdometryProcessor()
: Node("odometry_processor")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&OdometryProcessor::odom_callback, this, std::placeholders::_1));
  
 // Change from std_msgs::msg::Float64 to turtlebot3_msgs::msg::YawY
  robot_pose_pub_ = this->create_publisher<turtlebot3_msgs::msg::YawY>("robot_pose_yaw", qos);


  RCLCPP_INFO(this->get_logger(), "Odometry processor node has been initialized");
}

OdometryProcessor::~OdometryProcessor()
{
  RCLCPP_INFO(this->get_logger(), "Odometry processor node has been terminated");
}

void OdometryProcessor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  auto robot_pose_msg = turtlebot3_msgs::msg::YawY();
  robot_pose_msg.yaw = yaw;
  robot_pose_msg.y = msg->pose.pose.position.y;
  robot_pose_pub_->publish(robot_pose_msg);
}
