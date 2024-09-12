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
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_data_pub_;
};

#endif  // LASER_SCAN_PROCESSOR_HPP_

