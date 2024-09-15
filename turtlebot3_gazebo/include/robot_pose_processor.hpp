#ifndef ROBOT_POSE_PROCESSOR_HPP_
#define ROBOT_POSE_PROCESSOR_HPP_

#include <std_msgs/msg/float64.hpp>

class RobotPoseProcessor {
public:
  void update_pose(const std_msgs::msg::Float64::SharedPtr& msg);
  double get_yaw() const;

private:
  double yaw_;
};

#endif
