#include "opencv2/opencv.hpp"
#include <iostream>

namespace imageUndistort {

using namespace std;
using namespace cv;

class imageUndistort
{
   public:
        /** @brief Initializes the undistortion function.
        @param intrinsic camera intrinsic parameters, 3x3 Mat or 1x9 array.
        @param intrinsic camera distortion parameters, 1x9 Mat or array.
        @param image_size Image size.
        */
        imageUndistort(cv::Mat &intrinsic, cv::Mat &distortion, cv::Size image_size);
        imageUndistort(double intrinsic[], double distortion[], cv::Size image_size);
        imageUndistort(const imageUndistort &st);
        imageUndistort & operator=(const imageUndistort &st);
        imageUndistort(){}
        ~imageUndistort(){}

        /** @brief Undistort image.
        @param src Origin image.
        @param dst Undistortion image.
        */
        void doUndistort(cv::Mat & src, cv::Mat & dst);

     private:
        cv::Mat map_src_, map_dst_;
        void undistort(cv::Mat &intrinsic, cv::Mat &distortion, cv::Size image_size);
};
}