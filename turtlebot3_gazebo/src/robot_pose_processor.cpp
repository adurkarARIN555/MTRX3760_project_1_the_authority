#include "robot_pose_processor.hpp"

void RobotPoseProcessor::update_pose(const turtlebot3_msgs::msg::YawY::SharedPtr& msg) 
{
  yaw_ = msg->yaw;
  y_coordinate_ = msg->y;
}

double RobotPoseProcessor::get_yaw() const 
{ 
  return yaw_; 
}

double RobotPoseProcessor::get_y_coordinate() const 
{ 
  return y_coordinate_; 
}