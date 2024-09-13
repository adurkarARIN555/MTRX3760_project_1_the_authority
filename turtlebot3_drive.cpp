// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>
#include <cmath>  // Include for sqrt and pow for distance calculation

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node"),
  origin_reached_(false),  // Initialize flag as false
  initial_position_set_(false),  // Set this to false until the origin is captured
  origin_x_(0.0), origin_y_(0.0),  // Initialize origin positions to 0
  current_x_(0.0), current_y_(0.0)  // Initialize current positions to 0
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update the robot's current position (x, y)
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // Capture the origin position once (initial position)
  if (!initial_position_set_) {
    origin_x_ = current_x_;
    origin_y_ = current_y_;
    initial_position_set_ = true;  // Now we have the origin position set
  }

  // Extract yaw (robot's orientation) from the quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;  // Update the robot's yaw (orientation)
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  // Check if the origin has been reached
  if (origin_reached_) {
    // If origin is reached, stop the robot and shutdown ROS
    update_cmd_vel(0.0, 0.0);  // Stop the robot
    RCLCPP_INFO(this->get_logger(), "Maze solved! TurtleBot3 has returned to the origin.");
    rclcpp::shutdown();  // Stop the ROS node
    return;
  }

  // Calculate distance to the origin
  double distance_to_origin = std::sqrt(std::pow(current_x_ - origin_x_, 2) + std::pow(current_y_ - origin_y_, 2));
  double tolerance = 0.2;  // Tolerance to consider as reaching the origin

  if (distance_to_origin < tolerance) {
    // Mark as reached when the robot is within the tolerance
    origin_reached_ = true;
    return;
  }

  // Normal maze-solving logic (wall-following) continues below
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.7;
  double check_side_dist = 0.6;
  double check_left_dist = 0.3;

  switch (turtlebot3_state_num) {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist) {
        if (scan_data_[LEFT] < check_left_dist) {
          update_cmd_vel(LINEAR_VELOCITY, 0.0);  // Follow the left wall
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        } else {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;  // Turn left if there's no wall
        }
      } else {
        prev_robot_pose_ = robot_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;  // Turn right if blocked
      }
      break;

    case TB3_DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      }
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}

