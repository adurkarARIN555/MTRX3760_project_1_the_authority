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
