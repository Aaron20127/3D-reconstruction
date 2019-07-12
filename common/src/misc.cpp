#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>


namespace misc {

using namespace std;

void readImagesWithPattern(cv::String pattern, vector<cv::Mat> &images)
{
    std::vector<cv::String> fn;
    cv::glob(pattern, fn, false);

    for (const auto filename:fn) {
        images.push_back(cv::imread(filename));
    }
}

void readStereoImages(cv::String & left_image_pattern,
                      cv::String & right_image_pattern,
                      vector<cv::Mat> & left_images,
                      vector<cv::Mat> & right_images)
{
    std::vector<cv::String> left_fn, right_fn;
    cv::glob(left_image_pattern, left_fn, false);
    cv::glob(right_image_pattern, right_fn, false);

    for (auto left_path:left_fn) {
        cv::String left_filename = basename(left_path.c_str());

        for (auto right_path:right_fn) {
            cv::String right_filename = basename(right_path.c_str());

            if (left_filename == right_filename) {
                left_images.push_back(cv::imread(left_path));
                right_images.push_back(cv::imread(right_path));
            }

        }
    }
}

void formatPrintMat(cv::Mat mat, int width=26, int precision=17)
{      
    for(int i=0; i<mat.size().height; i++) {
        for(int j=0; j<mat.size().width; j++) {
            std::cout.precision(precision);
            std::cout.width(width);
            cout << std::left << mat.at<double>(i,j);
            if(j != mat.size().width-1)
                std::cout << "";
            else
                std::cout << endl; 
        }
    }
}

}