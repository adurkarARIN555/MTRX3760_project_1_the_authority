#include "laser_scan_processor.hpp"

LaserScanProcessor::LaserScanProcessor()
: Node("laser_scan_processor")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Create subscription to "scan" topic with SensorDataQoS
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanProcessor::ScanCallback, this, std::placeholders::_1));

  // Create publisher for processed sensor data
  sensor_data_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("sensor_data", qos);

  RCLCPP_INFO(this->get_logger(), "Laser scan processor node has been initialised");
}

LaserScanProcessor::~LaserScanProcessor()
{
  RCLCPP_INFO(this->get_logger(), "Laser scan processor node has been terminated");
}

void LaserScanProcessor::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std_msgs::msg::Float64MultiArray sensor_data;
  sensor_data.data.resize(3);

  // Process data at specific angles
  for (int num = 0; num < 3; num++) 
  {
    // Check for invalid readings and set to range_max if necessary
    sensor_data.data[num] = std::isinf(msg->ranges.at(scan_angle[num])) ? msg->range_max : msg->ranges.at(scan_angle[num]);
  }
  sensor_data_pub->publish(sensor_data);
}
