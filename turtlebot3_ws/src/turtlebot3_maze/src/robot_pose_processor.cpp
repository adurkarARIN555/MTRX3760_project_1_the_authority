// ***************************************
// src/robot_pose_processor.cpp
//
// Implementation for RobotPoseProcessor class
// ***************************************

#include "robot_pose_processor.hpp"

void RobotPoseProcessor::UpdatePose(const std_msgs::msg::Float64::SharedPtr& msg) 
{
  yaw = msg->data;
}

double RobotPoseProcessor::GetYaw() const 
{ 
  return yaw; 
}
