#ifndef MISC_H
#define MISC_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

namespace misc {

/** @brief Get all matrix of images from a file pattern.
@param pattern A path pattern like 'dir/*.png'.
@param images A vector that stores all image matrices.
*/
void readImagesWithPattern(cv::String pattern, std::vector<cv::Mat> &images);

/** @brief Get stereo image from two folder - 
 *         left camera images folder, right camera images folder.
@param left_image_pattern left images path pattern like 'dir/*.png'.
@param right_image_pattern right images path pattern like 'dir/*.png'.
@param left_images A vector that stores all image matrices.
@param right_images A vector that stores all image matrices.
*/
void readStereoImages(cv::String & left_image_pattern,
                      cv::String & right_image_pattern,
                      vector<cv::Mat> & left_images,
                      vector<cv::Mat> & right_images);

/** @brief print cv::Mat neatly.
@param mat opencv matrix.
@param width the print width.
@param precision all valid digits except 0 .
*/
void formatPrintMat(cv::Mat mat, int width=26, int precision=17);

}

#endif