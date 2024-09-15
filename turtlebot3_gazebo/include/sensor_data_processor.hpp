#ifndef SENSOR_DATA_PROCESSOR_HPP_
#define SENSOR_DATA_PROCESSOR_HPP_

#include <std_msgs/msg/float64_multi_array.hpp>

class SensorDataProcessor {
public:
  void update_sensor_data(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);

  double get_front_distance() const;
  double get_left_distance () const;
  double get_right_distance() const;

private:
  double scan_data_[3];
};

#endif
