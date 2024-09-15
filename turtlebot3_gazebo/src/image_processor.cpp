#include "image_processor.hpp"

void ImageProcessor::get_green_mask(const cv::Mat& image)
{
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, lower_green_, upper_green_, green_mask);
}

float ImageProcessor::calculate_green_percentage(const cv::Mat& image)
{
    // Update green_mask with the current image
    get_green_mask(image);

    int green_pixel_count = cv::countNonZero(green_mask);
    int total_pixels = image.rows * image.cols;

    return (green_pixel_count / (float)total_pixels) * 100.0f;
}

std::pair<double, double> ImageProcessor::find_green_centroid(const cv::Mat& image)
{
    // Update green_mask with the current image
    get_green_mask(image);

    cv::Moments moments = cv::moments(green_mask, true);

    std::pair<double, double> centroid = {0, 0};  // Default value

    if (moments.m00 > 0)
    {
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;
        centroid = {cx, cy};
    }

    return centroid;  // Return the result at the end
}


