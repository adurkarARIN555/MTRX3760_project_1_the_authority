// ***************************************
// src/image_processor.cpp
//
// Implementation for ImageProcessor class
// ***************************************

#include "image_processor.hpp"

void ImageProcessor::GetGreenMask(const cv::Mat& image)
{
    // Convert image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Create a binary mask where green colors are in the specified range
    cv::inRange(hsv_image, lower_green, upper_green, green_mask);
}

float ImageProcessor::CalculateGreenPercentage(const cv::Mat& image)
{
    // Update the mask to reflect the current image
    GetGreenMask(image);

    // Count the number of non-zero green pixels
    int green_pixel_count = cv::countNonZero(green_mask);
    int total_pixels = image.rows * image.cols;

    // Calculate and return the percentage of green pixels
    return (green_pixel_count / static_cast<float>(total_pixels)) * 100.0f;
}

std::pair<double, double> ImageProcessor::FindGreenCentroid(const cv::Mat& image)
{
    // Update the mask to reflect the current image
    GetGreenMask(image);

    // Compute the moments of the green mask to find the centroid
    cv::Moments moments = cv::moments(green_mask, true);

    std::pair<double, double> centroid = {0, 0};  

    if (moments.m00 > 0)
    {
        // Calculate centroid coordinates
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;
        centroid = {cx, cy};
    }

    return centroid;
}
