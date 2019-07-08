#ifndef MISC_H
#define MISC_H

#include <opencv2/opencv.hpp>
#include <iostream>

namespace misc {

/** @brief Get all matrix of images from a file pattern.
@param pattern A path pattern like 'dir/*.png'.
@param images A vector that stores all image matrices.
*/
void readImagesWithPattern(cv::String pattern, std::vector<cv::Mat> &images);

}

#endif