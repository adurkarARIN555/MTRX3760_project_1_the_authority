#include "turtlebot3_gazebo/turtlebot3_drive.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
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
    "scan", rclcpp::SensorDataQoS(), std::bind(
      &Turtlebot3Drive::scan_callback, this, std::placeholders::_1));
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
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
  robot_x_ = msg->pose.pose.position.y; // Update x-coordinate
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
  // Constants for distance thresholds and speeds
  const double DIST_THRESHOLD_FRONT = 0.35;    // Distance threshold for front wall
  const double DIST_THRESHOLD_LEFT = 0.35;     // Distance threshold for left wall
  const double DIST_THRESHOLD_RIGHT = 0.35;    // Distance threshold for right wall
  const double LINEAR_SPEED = 0.2;            // Linear speed
  const double TURN_SPEED = 1.5;              // Turning speed for sharper U-turns
  const double TURN_SPEED_SLOW = 0.6;         // Slower turning speed for wide turns
  const double TURN_SPEED_FAST_RIGHT = 1.5;   // Faster turning speed for tight right turns
  const double FINISH_LINE_X = 3.6;           // X-coordinate for finish line

  // Define sensor data for convenience
  double front_distance = scan_data_[0];
  double left_distance = scan_data_[1];
  double right_distance = scan_data_[2];

  // Initialize linear and angular speeds
  double linear_speed = LINEAR_SPEED;
  double angular_speed = 0.0;

  // Static variables for turn tracking
  //static bool turning = false;
  static rclcpp::Time turn_start_time;

  // Check if the robot has crossed the finish line
  if (robot_x_ >= FINISH_LINE_X) 
  {
    linear_speed = 0.0;
    angular_speed = 0.0;
    RCLCPP_INFO(this->get_logger(), "Robot has crossed the finish line.");
  } 
  // Decision making based on sensor data
  else if (left_distance < DIST_THRESHOLD_LEFT && front_distance < DIST_THRESHOLD_FRONT) 
  {
    angular_speed = -TURN_SPEED_FAST_RIGHT;
    linear_speed = 0.0;
    RCLCPP_INFO(this->get_logger(), "Inner corner detected: turning right sharply.");
  } 
  // else if (left_distance > DIST_THRESHOLD_LEFT) 
  // {
  //   // Outer corner detection: left wall has ended and no right wall
  //   linear_speed = LINEAR_SPEED;
  //   angular_speed = TURN_SPEED;
  //   RCLCPP_INFO(this->get_logger(), "Outer corner detected: making a left turn to follow wall.");
  // }
  else if (left_distance < DIST_THRESHOLD_LEFT) 
  {
    angular_speed = -TURN_SPEED;
    RCLCPP_INFO(this->get_logger(), "Wall on the left is too close: turning right.");
  } 
  else if (front_distance < DIST_THRESHOLD_FRONT) 
  {
    angular_speed = TURN_SPEED;
    RCLCPP_INFO(this->get_logger(), "Wall in front is too close: turning left.");
  } 
  else if (right_distance < DIST_THRESHOLD_RIGHT) 
  {
    linear_speed = LINEAR_SPEED;
    angular_speed = TURN_SPEED_SLOW;
    RCLCPP_INFO(this->get_logger(), "Wall on the right: moving forward with slight left turn.");
  } 
  else 
  {
    linear_speed = LINEAR_SPEED;
    angular_speed = TURN_SPEED_SLOW;
    RCLCPP_INFO(this->get_logger(), "No significant obstacles: moving forward with slight left turn.");
  }

  // Update robot velocity
  update_cmd_vel(linear_speed, angular_speed);
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
