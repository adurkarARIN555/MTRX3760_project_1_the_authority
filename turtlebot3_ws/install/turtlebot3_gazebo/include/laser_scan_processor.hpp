#ifndef LASER_SCAN_PROCESSOR_HPP_
#define LASER_SCAN_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class LaserScanProcessor : public rclcpp::Node
{
public:
   LaserScanProcessor();
  ~LaserScanProcessor();

private:
  // Angles of interest in degrees for processing laser scan data
  uint16_t scan_angle[3] = {0, 30, 330};

  // Callback function to process laser scan data
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // ROS subscription for laser scan data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

  // ROS publisher for processed sensor data
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_data_pub;
};

#endif

