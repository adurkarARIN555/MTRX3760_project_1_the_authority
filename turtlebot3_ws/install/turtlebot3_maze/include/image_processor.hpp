// ********************************************************************************
// include/image_processor.hpp
//
// This header file defines the ImageProcessor class, which provides utilities
// for processing images to detect green color. The class calculates the percentage
// of green pixels in an image and identifies the centroid of the detected green 
// object. These methods are used by nodes, such as ColorDetector, to interpret 
// camera data and assist with robot navigation.
//
// Author Info: Eashan Garg, Arin Adurkar, Savith Karunanayaka, Joel Pivetta
// Year: 2024
// ********************************************************************************

#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <cv_bridge/cv_bridge.h>

class ImageProcessor
{
public:
    // Returns the percentage of green pixels in an image
    float CalculateGreenPercentage(const cv::Mat& image);

    // Returns the co-ordinates of the green pixels centroid in an image
    std::pair<double, double> FindGreenCentroid(const cv::Mat& image);

private:
    cv::Mat green_mask;

    // Creates a mask identifying green pixels in the image
    void GetGreenMask(const cv::Mat& image);
    
    // Lower bound for green color in HSV
    cv::Scalar lower_green = cv::Scalar(35, 100, 100);

    // Upper bound for green color in HSV
    cv::Scalar upper_green = cv::Scalar(85, 255, 255);
};
#endif