#ifndef CAMERA_H
#define CAMERA_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h> 
#include <unistd.h>
#include "debug.h"
#include "timing.h"
#include "imageUndistort.h"

namespace  camera {
    using namespace imageUndistort;
    using namespace std;
    using namespace cv;
    using namespace timing;
}

namespace camera {

class camera : public imageUndistort
{
   public:
        typedef std::pair<cv::Mat, cv::VideoCapture> ImageVediocapturePairType;
        enum {USB=0, RTSP=1};

        /** @brief Grab RSTP network protocol camera of hikvision.
        @param user User name.
        @param passwd Passwd.
        @param passwd Passwd.
        @param ip Ip address, such as "192.168.1.1".
        @param intrinsic Camera intrinsic parameters, 3x3 Mat or 1x9 array.
        @param intrinsic Camera distortion parameters, 1x9 Mat or array.
        */
        camera(cv::String & user, cv::String & passwd, cv::String & ip);
        camera(cv::String & user, cv::String & passwd, cv::String & ip, cv::Mat &intrinsic, cv::Mat &distortion);
        camera(cv::String & user, cv::String & passwd, cv::String & ip, double intrinsic[], double distortion[]);

        /** @brief Grab usb protocol camera.
        @param id Camera driver id, such as 0.
        @param intrinsic camera intrinsic parameters, 3x3 Mat or 1x9 array.
        @param intrinsic camera distortion parameters, 1x9 Mat or array.
        */
        camera(int id);
        camera(int id, cv::Mat &intrinsic, cv::Mat &distortion);
        camera(int id, double intrinsic[], double distortion[]);

        camera();
        /** @brief Note: copy constructor will not start thread. */
        camera(const camera & st);
        /** @brief Note: assignment operation will copy all of state. */
        camera & operator=(const camera &st);
        ~camera();

        /** @brief Thread function. */
        static void * threadCaptureCamera(void * params);
        /** @brief Start grab thread. */
        void start();
        /** @brief Stop grab thread. */
        void stop();
        /** @brief Get image without undistortion. */
        void originImage(cv::Mat & dst);
        /** @brief Get image with undistortion. */
        void undistortImage(cv::Mat & dst);
        /** @brief Get camera id, such as, RSTP string "192.168.1.1", USB int 0. */
        cv::String id();

    private:        
        int addr_id_;
        cv::String addr_url_;
        cv::String ip_;
        cv::Size image_size_;
        int camera_type_;
        cv::VideoCapture capture_;
        bool undistort_;
        bool thread_run_;
        pthread_t  pthread_;
        ImageVediocapturePairType *image_capture_pair_;

        // init
        void initRTSP(cv::String & user, cv::String & passwd, cv::String & ip);
        void initUSB(int & id);

        cv::Size imageSize(cv::VideoCapture & capture);
        cv::Size imageSize(cv::String & user, cv::String & passwd, cv::String & ip);
        cv::Size imageSize(int &id);
        cv::VideoCapture getCapture(cv::String & url);
        cv::VideoCapture getCapture(int & id);
};

} //camera

#endif