#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <cv_bridge/cv_bridge.h>

class ImageProcessor
{
public:
    float calculate_green_percentage(const cv::Mat& image);
    std::pair<double, double> find_green_centroid(const cv::Mat& image);

private:
    cv::Mat green_mask;
    void get_green_mask(const cv::Mat& image);
    
    cv::Scalar lower_green_ = cv::Scalar(35, 100, 100);
    cv::Scalar upper_green_ = cv::Scalar(85, 255, 255);
};

#endif
