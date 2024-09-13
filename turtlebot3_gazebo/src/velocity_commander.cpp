#include "velocity_commander.hpp"

geometry_msgs::msg::Twist VelocityCommander::generate_velocity_command(double front_dist, double left_dist, double right_dist, double robot_y) 
{
    const double DIST_THRESHOLD_FRONT = 0.35;
    const double DIST_THRESHOLD_LEFT = 0.35;
    const double DIST_THRESHOLD_RIGHT = 0.35;
    const double LINEAR_SPEED = 0.2;
    const double TURN_SPEED = 1.5;
    const double TURN_SPEED_SLOW = 0.6;
    const double TURN_SPEED_FAST_RIGHT = 1.5;

    geometry_msgs::msg::Twist cmd_vel;
    double linear_speed = LINEAR_SPEED;
    double angular_speed = 0.0;

    if (robot_y >= FINISH_LINE_Y) 
    {
      linear_speed = 0.0;
      angular_speed = 0.0;
    } 
    else if (left_dist < DIST_THRESHOLD_LEFT && front_dist < DIST_THRESHOLD_FRONT) 
    {
      angular_speed = -TURN_SPEED_FAST_RIGHT;
      linear_speed = 0.0;
    } 
    else if (left_dist < DIST_THRESHOLD_LEFT) 
    {
      angular_speed = -TURN_SPEED;
    } 
    else if (front_dist < DIST_THRESHOLD_FRONT) 
    {
      angular_speed = TURN_SPEED;
    } 
    else if (right_dist < DIST_THRESHOLD_RIGHT) 
    {
      linear_speed = LINEAR_SPEED;
      angular_speed = TURN_SPEED_SLOW;
    } 
    else 
    {
      linear_speed = LINEAR_SPEED;
      angular_speed = TURN_SPEED_SLOW;
    }

    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_speed;
    return cmd_vel;
  }