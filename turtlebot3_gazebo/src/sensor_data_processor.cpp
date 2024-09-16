#include "sensor_data_processor.hpp"

void SensorDataProcessor::UpdateSensorData(const std_msgs::msg::Float64MultiArray::SharedPtr& msg) 
{
  scan_data[0] = msg->data[0]; // Front distance
  scan_data[1] = msg->data[1]; // Left distance
  scan_data[2] = msg->data[2]; // Right distance
}

double SensorDataProcessor::GetFrontDistance() const 
{ 
  return scan_data[0]; 
}
double SensorDataProcessor::GetLeftDistance()  const 
{ 
  return scan_data[1]; 
}
double SensorDataProcessor::GetRightDistance() const 
{ 
  return scan_data[2]; 
}