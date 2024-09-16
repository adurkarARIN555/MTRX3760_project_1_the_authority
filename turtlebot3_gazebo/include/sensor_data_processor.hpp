#ifndef SENSOR_DATA_PROCESSOR_HPP_
#define SENSOR_DATA_PROCESSOR_HPP_

#include <std_msgs/msg/float64_multi_array.hpp>

class SensorDataProcessor 
{
public:
  // Updates internal sensor data with the incoming message
  void UpdateSensorData(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);

  // Returns the distance measured by the front, left and right sensors
  double GetFrontDistance() const;
  double GetLeftDistance () const;
  double GetRightDistance() const;

private:
  // Array stores distances from front, left, and right sensors
  double scan_data[3];
};

#endif
