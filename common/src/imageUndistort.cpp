
#include "imageUndistort.h"

namespace imageUndistort {

imageUndistort::imageUndistort(const imageUndistort &st)
{
    map_src_ = st.map_src_.clone();
    map_dst_ = st.map_dst_.clone();
}   

imageUndistort & imageUndistort::operator=(const imageUndistort &st) 
{
    map_src_ = st.map_src_.clone();
    map_dst_ = st.map_dst_.clone();
    return *this;
}

imageUndistort::imageUndistort(cv::Mat &intrinsic, cv::Mat &distortion, cv::Size image_size)
{
    undistort(intrinsic, distortion, image_size);
}

imageUndistort::imageUndistort(double intrinsic[], double distortion[], cv::Size image_size)
{
    cv::Mat intrinsic_tmp(3, 3, CV_64FC1, intrinsic);
    cv::Mat distortion_tmp(1, 5, CV_64FC1, distortion);

    undistort(intrinsic_tmp, distortion_tmp, image_size);
}

void imageUndistort::undistort(cv::Mat &intrinsic, cv::Mat &distortion, cv::Size image_size)
{
    cv::Mat empty_matrix;
    cv::initUndistortRectifyMap(intrinsic, distortion, empty_matrix, 
                                intrinsic, image_size, CV_32FC1,
                                map_src_, map_dst_);
}

void imageUndistort::doUndistort(cv::Mat & src, cv::Mat & dst)
{
    cv::remap(src, dst, map_src_, map_dst_, cv::INTER_LINEAR);
}

}