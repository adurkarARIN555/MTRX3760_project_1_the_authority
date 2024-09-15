#ifndef COLOR_DETECTOR_HPP
#define COLOR_DETECTOR_HPP

#include "image_processor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>



class ColorDetector : public rclcpp::Node
{
public:
     ColorDetector();
    ~ColorDetector();

private:
    void image_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void publish_green_goal_position(double x_offset, double y_offset);

    // ROS publishers and subscriptions
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr green_percentage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr green_goal_pos_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // Image processing utility
    ImageProcessor image_processor_;

    float green_percentage;
    std::pair<double, double> green_centroid_;
};

#endif
