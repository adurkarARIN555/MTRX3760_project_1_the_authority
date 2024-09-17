// ********************************************************************************
// include/color_detector.hpp
//
// This header file defines the ColorDetector class, a ROS 2 node that processes
// incoming images to detect the presence of green objects. The node calculates
// the percentage of green in the image and determines the position (centroid) of
// the green object. This information is then published to ROS topics for further
// use in controlling the TurtleBot3.
//
// Author Info: Eashan Garg, Arin Adurkar, Savith Karunanayaka, Joel Pivetta
// Year: 2024
// ********************************************************************************

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
    // Callback for processing incoming images
    void ImageCallback(sensor_msgs::msg::Image::SharedPtr msg);

    // Publish the green object's position (centroid)
    void PublishGreenGoalPosition(double x_offset, double y_offset);

    // ROS publishers and subscriptions
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr green_percentage_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr green_goal_pos_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;

    // Image processing utility
    ImageProcessor image_processor;

    // Green color detection results
    float green_percentage;
    std::pair<double, double> green_centroid;
};

#endif
