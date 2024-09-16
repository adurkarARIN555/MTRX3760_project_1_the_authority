#include <rclcpp/rclcpp.hpp>
#include "laser_scan_processor.hpp"
#include "odom_processor.hpp"
#include "obstacle_avoidance.hpp"
#include "color_detector.hpp"

// Function to initialise and return a list of nodes
std::vector<std::shared_ptr<rclcpp::Node>> initialize_nodes() 
{
  auto laser_scan_processor = std::make_shared<LaserScanProcessor>();
  auto odom_processor = std::make_shared<OdometryProcessor>();
  auto obstacle_avoidance = std::make_shared<ObstacleAvoidance>();
  auto color_detector = std::make_shared<ColorDetector>();

  return {laser_scan_processor, odom_processor, obstacle_avoidance, color_detector};
}

// Function to setup and spin the executor
void setup_executor(const std::vector<std::shared_ptr<rclcpp::Node>>& nodes) 
{
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add each node to the executor
  for (const auto& node : nodes) 
  {
    executor.add_node(node);
  }
  executor.spin();
}

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);

  auto nodes = initialize_nodes();
  setup_executor(nodes);

  rclcpp::shutdown();
  return 0;
}

