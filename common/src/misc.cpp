#include <opencv2/opencv.hpp>
#include <iostream>


namespace misc {

void readImagesWithPattern(cv::String pattern, std::vector<cv::Mat> &images)
{
    std::vector<cv::String> fn;
    cv::glob(pattern, fn, false);

    for (const auto filename:fn) {
        images.push_back(cv::imread(filename));
    }
}

}