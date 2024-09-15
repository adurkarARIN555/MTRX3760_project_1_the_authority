#ifndef VELOCITY_COMMANDER_HPP_
#define VELOCITY_COMMANDER_HPP_

#include <geometry_msgs/msg/twist.hpp>

class VelocityCommander {
public:
  VelocityCommander() : FINISH_LINE_Y(3.6) {}

  geometry_msgs::msg::Twist generate_velocity_command(double front_dist, double left_dist, double right_dist, double green_percentage, double green_goal_x);

private:
  const double FINISH_LINE_Y;
};

#endif  // VELOCITY_COMMANDER_HPP_
