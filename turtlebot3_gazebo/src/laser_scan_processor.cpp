#include "laser_scan_processor.hpp"

LaserScanProcessor::LaserScanProcessor()
: Node("laser_scan_processor")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanProcessor::scan_callback, this, std::placeholders::_1));
  sensor_data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("sensor_data", qos);

  RCLCPP_INFO(this->get_logger(), "Laser scan processor node has been initialized");
}

LaserScanProcessor::~LaserScanProcessor()
{
  RCLCPP_INFO(this->get_logger(), "Laser scan processor node has been terminated");
}

void LaserScanProcessor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};
  std_msgs::msg::Float64MultiArray sensor_data;
  sensor_data.data.resize(3);

  for (int num = 0; num < 3; num++) {
    sensor_data.data[num] = std::isinf(msg->ranges.at(scan_angle[num])) ? msg->range_max : msg->ranges.at(scan_angle[num]);
  }

  sensor_data_pub_->publish(sensor_data);
}

