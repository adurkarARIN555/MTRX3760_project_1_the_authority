#include "color_detector.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

ColorDetector::ColorDetector()
: Node("color_detector")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10, std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));

    green_percentage_pub_ = this->create_publisher<std_msgs::msg::Float32>("green_percentage", 10);
    green_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("green_detected", 10);

    RCLCPP_INFO(this->get_logger(), "Color detector node has been initialized");
}

void ColorDetector::image_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat image = cv_ptr->image;
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the range for green color
    cv::Scalar lower_green(35, 100, 100);
    cv::Scalar upper_green(85, 255, 255);
    cv::Mat green_mask;
    cv::inRange(hsv_image, lower_green, upper_green, green_mask);

    // Count the number of green pixels
    int green_pixel_count = cv::countNonZero(green_mask);
    int total_pixels = image.rows * image.cols;
    float green_percentage = (green_pixel_count / (float)total_pixels) * 100.0f;

    // Publish the green percentage
    std_msgs::msg::Float32 green_percentage_msg;
    green_percentage_msg.data = green_percentage;
    green_percentage_pub_->publish(green_percentage_msg);

    // Publish whether green is detected (above a certain threshold)
    std_msgs::msg::Bool green_detected_msg;
    green_detected_msg.data = (green_pixel_count > 0);
    green_detected_pub_->publish(green_detected_msg);

    // Optionally, print the detection info to the terminal
    if (green_detected_msg.data)
    {
        RCLCPP_INFO(this->get_logger(), "Green color detected: %.2f%%", green_percentage);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No green color detected");
    }
}



