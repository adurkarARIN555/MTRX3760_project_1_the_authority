#include <rclcpp/rclcpp.hpp>
#include "laser_scan_processor.hpp"
#include "odom_processor.hpp"
#include "obstacle_avoidance.hpp"
#include "color_detector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto laser_scan_processor = std::make_shared<LaserScanProcessor>();
  auto odom_processor = std::make_shared<OdometryProcessor>();
  auto obstacle_avoidance = std::make_shared<ObstacleAvoidance>();
  auto color_detector = std::make_shared<ColorDetector>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(laser_scan_processor);
  executor.add_node(odom_processor);
  executor.add_node(obstacle_avoidance);
  executor.add_node(color_detector); 

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
