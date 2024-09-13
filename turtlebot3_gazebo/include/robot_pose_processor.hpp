#ifndef ROBOT_POSE_PROCESSOR_HPP_
#define ROBOT_POSE_PROCESSOR_HPP_

#include <turtlebot3_msgs/msg/yaw_y.hpp>

class RobotPoseProcessor {
public:
  void update_pose(const turtlebot3_msgs::msg::YawY::SharedPtr& msg);

  double get_yaw() const;
  double get_y_coordinate() const;

private:
  double yaw_;
  double y_coordinate_;
};

#endif  // ROBOT_POSE_PROCESSOR_HPP_
