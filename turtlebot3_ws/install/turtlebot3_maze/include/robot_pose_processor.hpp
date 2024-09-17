// ********************************************************************************
// include/robot_pose_processor.hpp
//
// This header file defines the RobotPoseProcessor class, which is responsible for
// managing and updating the robot's pose (specifically the yaw angle). It provides 
// methods to update the pose using incoming messages and retrieve the current yaw 
// angle for navigation or control purposes.
//
// Author Info: Eashan Garg, Arin Adurkar, Savith Karunanayaka, Joel Pivetta
// Year: 2024
// ********************************************************************************

#ifndef ROBOT_POSE_PROCESSOR_HPP_
#define ROBOT_POSE_PROCESSOR_HPP_

#include <std_msgs/msg/float64.hpp>

class RobotPoseProcessor 
{
public:
  // Updates the internal pose data with the incoming message
  void UpdatePose(const std_msgs::msg::Float64::SharedPtr& msg);

  // Returns the current yaw angle
  double GetYaw() const;

private:
  double yaw;
};
#endif