#include "sensor_data_processor.hpp"

void SensorDataProcessor::update_sensor_data(const std_msgs::msg::Float64MultiArray::SharedPtr& msg) 
{
  scan_data_[0] = msg->data[0]; // Front distance
  scan_data_[1] = msg->data[1]; // Left distance
  scan_data_[2] = msg->data[2]; // Right distance
}

double SensorDataProcessor::get_front_distance() const 
{ 
  return scan_data_[0]; 
}
double SensorDataProcessor::get_left_distance() const 
{ 
  return scan_data_[1]; 
}
double SensorDataProcessor::get_right_distance() const 
{ 
  return scan_data_[2]; 
}