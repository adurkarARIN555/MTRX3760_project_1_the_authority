#include "color_detector.hpp"

ColorDetector::ColorDetector()
: Node("color_detector")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", 10, std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));

    green_percentage_pub_ = this->create_publisher<std_msgs::msg::Float64>("green_color_percentage", 10);
    green_goal_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("green_goal_position", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Color detector node has been initialized");
}

ColorDetector::~ColorDetector()
{
    RCLCPP_INFO(this->get_logger(), "Color detector node has been terminated");
}

void ColorDetector::publish_green_goal_position(double x_offset, double y_offset)
{
    // Publishes the goal position in the camera frame
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {x_offset, y_offset};

    green_goal_pos_pub_->publish(msg);
}

void ColorDetector::image_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // Use the ImageProcessor instance to calculate the green percentage
    green_percentage = image_processor_.calculate_green_percentage(image);

    // Publish the green percentage
    std_msgs::msg::Float64 green_percentage_msg;
    green_percentage_msg.data = green_percentage;
    green_percentage_pub_->publish(green_percentage_msg);

    // Use ImageProcessor to find the centroid and store it in the class variable
    green_centroid_ = image_processor_.find_green_centroid(image);

    // Check if the centroid is valid (not (0, 0)), and if so, publish the goal position
    if (green_centroid_.first != 0 || green_centroid_.second != 0)
    {
        double x_offset = green_centroid_.first - (image.cols / 2);
        double y_offset = green_centroid_.second - (image.rows / 2);
        publish_green_goal_position(x_offset, y_offset);
    }
}

