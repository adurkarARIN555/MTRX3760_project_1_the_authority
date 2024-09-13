#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

// Global variables to store the current position
float x = 0;
float y = 0;
float z = 0;

class PointsAndLinesNode : public rclcpp::Node
{
public:
  PointsAndLinesNode()
  : Node("points_and_lines")
  {
    // Publisher for visualization markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    
    // Subscriber to Odometry topic
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&PointsAndLinesNode::topic_callback, this, std::placeholders::_1)
    );

    // Set rate of marker publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 10Hz
      std::bind(&PointsAndLinesNode::publish_markers, this)
    );
  }

private:
  // Callback for Odometry messages
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Access position data from the Odometry message
    const auto& position = msg->pose.pose.position;
    
    // Update global variables with received data
    x = position.x;
    y = position.y;
    z = position.z;
    
    RCLCPP_INFO(this->get_logger(), "Received Odometry: x=%.2f, y=%.2f, z=%.2f", x, y, z);
  }

  void publish_markers()
  {
    // Declare the points and line_strip markers
    static visualization_msgs::msg::Marker points, line_strip;

    // Set marker properties (shared across points and line_strip)
    points.header.frame_id = line_strip.header.frame_id = "odom";
    points.header.stamp = line_strip.header.stamp = this->now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::msg::Marker::POINTS;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

    points.scale.x = 0.1;
    points.scale.y = 0.1;
    line_strip.scale.x = 0.05;

    // Set color for points and lines
    points.color.g = 1.0f;
    points.color.a = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Add the current position as a new point to both points and line_strip
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    points.points.push_back(p);
    line_strip.points.push_back(p);

    // Publish both points and line_strip
    marker_pub_->publish(points);
    marker_pub_->publish(line_strip);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointsAndLinesNode>());
  rclcpp::shutdown();
  return 0;
}
