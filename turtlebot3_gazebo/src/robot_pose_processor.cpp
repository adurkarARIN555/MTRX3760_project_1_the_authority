#include "robot_pose_processor.hpp"

void RobotPoseProcessor::update_pose(const std_msgs::msg::Float64::SharedPtr& msg) 
{
  yaw_ = msg->data;
}

double RobotPoseProcessor::get_yaw() const 
{ 
  return yaw_; 
}
