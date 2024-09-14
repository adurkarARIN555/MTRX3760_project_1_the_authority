#ifndef COLOR_DETECTOR_HPP
#define COLOR_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class ColorDetector : public rclcpp::Node
{
public:
    ColorDetector();

private:
    void image_callback(sensor_msgs::msg::Image::SharedPtr msg);

    // Add the publishers for green percentage and detection status
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr green_percentage_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr green_detected_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

#endif // COLOR_DETECTOR_HPP

