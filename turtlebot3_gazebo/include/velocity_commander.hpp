#ifndef VELOCITY_COMMANDER_HPP_
#define VELOCITY_COMMANDER_HPP_

#include <geometry_msgs/msg/twist.hpp>

class VelocityCommander {
public:
  VelocityCommander() : linear_speed(LINEAR_SPEED), angular_speed(NO_MOVEMENT) {}
  geometry_msgs::msg::Twist generate_velocity_command(double front_dist, double left_dist, double right_dist, double green_percentage, double green_goal_x);

private:
  // Methods for different behaviors
  void handle_green_detection(double green_goal_x, double& linear_speed, double& angular_speed);
  void handle_wall_avoidance(double front_dist, double left_dist, double right_dist, double& linear_speed, double& angular_speed);

  // Constants
  const double DIST_THRESHOLD_FRONT    = 0.35;
  const double DIST_THRESHOLD_LEFT     = 0.35;
  const double DIST_THRESHOLD_RIGHT    = 0.35;
  const double LINEAR_SPEED            = 0.2;
  const double TURN_SPEED              = 1.5;
  const double TURN_SPEED_SLOW         = 0.6;
  const double GOAL_MET_THRESHOLD      = 91.0;
  const double GOAL_DETECTED_THRESHOLD = 5.0;
  const double NO_MOVEMENT             = 0.0;
  const double GOAL_CENTRING_POS       = 0.1;
  const double GOAL_TURN_SPEED         = 0.5;

  double linear_speed;
  double angular_speed;

};

#endif
