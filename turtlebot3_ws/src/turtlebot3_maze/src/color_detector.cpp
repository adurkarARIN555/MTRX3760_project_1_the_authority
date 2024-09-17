// ***************************************
// src/color_detector.cpp
//
// Implementation for ColorDetector class
// ***************************************

#include "color_detector.hpp"

ColorDetector::ColorDetector() : Node("color_detector")
{
    // Create a subscription to the "camera/image_raw" topic with a queue size of 10
    subscription = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", 10, std::bind(&ColorDetector::ImageCallback, this, std::placeholders::_1));

    // Create a publisher for green color percentage with a queue size of 10
    green_percentage_pub = this->create_publisher<std_msgs::msg::Float64>("green_color_percentage", 10);

    // Create a publisher for green goal position with a QOS profile
    green_goal_pos_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("green_goal_position", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Color detector node has been initialised");
}

ColorDetector::~ColorDetector()
{
    RCLCPP_INFO(this->get_logger(), "Color detector node has been terminated");
}

void ColorDetector::PublishGreenGoalPosition(double x_offset, double y_offset)
{
    // Publishes the goal position in the camera frame
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {x_offset, y_offset};

    green_goal_pos_pub->publish(msg);
}

void ColorDetector::ImageCallback(sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // Use the ImageProcessor instance to calculate the green percentage
    green_percentage = image_processor.CalculateGreenPercentage(image);

    // Publish the green percentage
    std_msgs::msg::Float64 green_percentage_msg;
    green_percentage_msg.data = green_percentage;
    green_percentage_pub->publish(green_percentage_msg);

    // Use ImageProcessor to find the centroid and store it in the class variable
    green_centroid = image_processor.FindGreenCentroid(image);

    // Check if the centroid is valid (not (0, 0)), and if so, publish the goal position
    if (green_centroid.first != 0 || green_centroid.second != 0)
    {
        double x_offset = green_centroid.first - (image.cols / 2);
        double y_offset = green_centroid.second - (image.rows / 2);
        PublishGreenGoalPosition(x_offset, y_offset);
    }

    // Debugging output for green_percentage
    // RCLCPP_INFO(this->get_logger(), "Green color percentage: %.2f%%", green_percentage);
}

