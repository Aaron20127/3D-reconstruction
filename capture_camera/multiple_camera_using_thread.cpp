
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h> 
#include <unistd.h>
#include <chrono>
#include "debug.h"

namespace CC {

using namespace std;
using namespace cv;

#define TIME_COUNT (std::chrono::duration_cast<std::chrono::duration<double>>(\
                    std::chrono::steady_clock::now().time_since_epoch())).count()

typedef std::pair<cv::Mat, cv::VideoCapture> ImageVediocapturePairType;

class imageUndistort
{
   private:
        cv::Mat map_src_, map_dst_;
        void undistort(cv::Mat &intrinsic, cv::Mat &distortion, cv::Size image_size);

   public:
        void doUndistort(cv::Mat & src, cv::Mat & dst);

        imageUndistort(){}
        imageUndistort(cv::Mat &intrinsic, cv::Mat &distortion, cv::Size image_size);
        imageUndistort(double intrinsic[], double distortion[], cv::Size image_size);

        imageUndistort(const imageUndistort & st)
        {
            cout << "imageUndistort(const imageUndistort & st)" << endl;
            map_src_ = st.map_src_.clone();
            map_dst_ = st.map_dst_.clone();
        }   

        imageUndistort & operator=(const imageUndistort &st) 
        {
            cout << "imageUndistort & operator=(const imageUndistort &st)" << endl;
            map_src_ = st.map_src_.clone();
            map_dst_ = st.map_dst_.clone();
            return *this;
        }
        ~imageUndistort(){}
};

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
    cv::remap(src, dst, map_src_, map_dst_, INTER_LINEAR);
}

/*********************************camera**********************************************/
class camera : public imageUndistort
{
   private:
        enum {USB=0, RTSP=1};

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

   public:
        // RSTP camera
        camera(cv::String & user, cv::String & passwd, cv::String & ip);
        camera(cv::String & user, cv::String & passwd, cv::String & ip, 
                    cv::Mat &intrinsic, cv::Mat &distortion);
        camera(cv::String & user, cv::String & passwd, cv::String & ip,
                    double intrinsic[], double distortion[]);

        // USB camera
        camera(int id);
        camera(int id, cv::Mat &intrinsic, cv::Mat &distortion);
        camera(int id, double intrinsic[], double distortion[]);

        // 
        camera(){
            pthread_ = 0;
            thread_run_ = false;
            image_capture_pair_ = NULL;
        };

        // copy destructor, Note: new camera class don't run thread !!!
        camera(const camera & st) : imageUndistort(st)
        {   
            // copy value 
            addr_id_ = st.addr_id_;
            ip_ = st.ip_;
            addr_url_ = st.addr_url_;
            image_size_ = st.image_size_;
            camera_type_ = st.camera_type_;
            undistort_ = st.undistort_;
            thread_run_ = false;
            pthread_ = st.pthread_;


            // new
            if (camera_type_ == RTSP) {
                capture_ = getCapture(addr_url_);
            } else {
                capture_ = st.capture_;
            }
            cv::Mat image(image_size_, CV_8UC3);
            image_capture_pair_ = new ImageVediocapturePairType (image.clone(), capture_);


            cout << "copy destructor, id " << id() << endl;  
        }

        // assignment
        camera & operator=(const camera &st)
        {
            cout << "assignment, id " << id() << endl;  

            // stop
            stop();
            delete image_capture_pair_;

            // base class
            imageUndistort::operator=(st);

            // copy value 
            addr_id_ = st.addr_id_;
            ip_ = st.ip_;
            addr_url_ = st.addr_url_;
            image_size_ = st.image_size_;
            camera_type_ = st.camera_type_;
            undistort_ = st.undistort_;
            thread_run_ = st.thread_run_;
            pthread_ = st.pthread_;

            // new
            if (camera_type_ == RTSP) {
                capture_ = getCapture(addr_url_);
            } else {
                capture_ = st.capture_;
            }

            cv::Mat image(image_size_, CV_8UC3);
            image_capture_pair_ = new ImageVediocapturePairType (image.clone(), capture_);

            // start
            if (thread_run_) {
                start();
            }
            return *this;
        }

        ~camera()
        {
            cout << "delete, id " << id() << endl;
            stop();
            // capture_.release();
            delete image_capture_pair_;
        }

        // function
        static void * threadCaptureCamera(void * params);
        void start();
        void stop();
        void originImage(cv::Mat & dst);
        void undistortImage(cv::Mat & dst);
        cv::String id();
};

cv::Size camera::imageSize(cv::String & user, cv::String & passwd, cv::String & ip)
{
    cv::String url = cv::String("rtsp://") + user + ':' + passwd +
                      "@" + ip + "//Streaming/Channels/1";
    cv::VideoCapture capture = getCapture(url);
    return imageSize(capture);
}

cv::Size camera::imageSize(int &id)
{
    cv::VideoCapture capture = getCapture(id);
    return imageSize(capture);
}


cv::Size camera::imageSize(cv::VideoCapture & capture)
{
    cv::Mat image;
    capture >> image;
    while (image.empty()) {
        cout << "image is empty, recap" <<endl;
        capture >> image;
    }
    return image.size();
}

void camera::initRTSP(cv::String & user, cv::String & passwd, cv::String & ip)
{
    ip_ = ip;
    camera_type_ = RTSP;
    pthread_ = 0;
    thread_run_ = false;

    addr_url_ = cv::String("rtsp://") + user + ':' + passwd +
                      "@" + ip + "//Streaming/Channels/1";
    capture_ = getCapture(addr_url_);
    image_size_ = imageSize(capture_);

    cv::Mat image(image_size_, CV_8UC3);
    image_capture_pair_ = new ImageVediocapturePairType (image.clone(), capture_);
}

void camera::initUSB(int & id)
{
    addr_id_ = id;
    camera_type_ = USB;
    pthread_ = 0;
    thread_run_ = false;

    capture_ = getCapture(addr_id_);
    image_size_ = imageSize(capture_);

    cv::Mat image(image_size_, CV_8UC3);
    image_capture_pair_ = new ImageVediocapturePairType (image.clone(), capture_);
}

/* camera RTSP constructor */
camera::camera(cv::String & user, cv::String & passwd, cv::String & ip)
    : imageUndistort()
{
    initRTSP(user, passwd, ip);
    undistort_ = false;
}


camera::camera(cv::String & user, cv::String & passwd, cv::String & ip, 
                        double intrinsic[], double distortion[])
    : imageUndistort(intrinsic, distortion, imageSize(user, passwd, ip))
{
    initRTSP(user, passwd, ip);
    undistort_ = true;
}


camera::camera(cv::String & user, cv::String & passwd, cv::String & ip, 
                        cv::Mat &intrinsic, cv::Mat &distortion)
    : imageUndistort(intrinsic, distortion, imageSize(user, passwd, ip))
{
    initRTSP(user, passwd, ip);
    undistort_ = true;
}

/* camera USB constructor */
camera::camera(int id)
    : imageUndistort()
{
    initUSB(id);
    undistort_ = false;
}


camera::camera(int id, double intrinsic[], double distortion[])
    : imageUndistort(intrinsic, distortion, imageSize(id))
{
    initUSB(id);
    undistort_ = true;
}


camera::camera(int id, cv::Mat &intrinsic, cv::Mat &distortion)
    : imageUndistort(intrinsic, distortion, imageSize(id))
{
    initUSB(id);
    undistort_ = true;
}

cv::VideoCapture camera::getCapture(cv::String & url)
{
    VideoCapture capture = cv::VideoCapture(url);
    if (!capture.isOpened()) {
        cerr << "ERROR! Unable to open camera: " << url << endl;
        exit(-1);
    }

    return capture;
}

cv::VideoCapture camera::getCapture(int & id)
{
    VideoCapture capture = cv::VideoCapture(id);
    if (!capture.isOpened()) {
        cerr << "ERROR! Unable to open camera: " << id << endl;
        exit(-1);
    }

    return capture;
}


/* function */
void * camera::threadCaptureCamera(void * params)
{
    ImageVediocapturePairType * image_cap = 
        (ImageVediocapturePairType * )params;
    cv::Mat dst(image_cap->first);
    cv::VideoCapture cap(image_cap->second);

    /* Continuous capture */
    double t = TIME_COUNT;
    int count = 0;
    while(true) {
        count ++;
        t = TIME_COUNT;
        cap.grab();
        cap.retrieve(dst);
        cout << "pt " <<  params << ", count " << count  
             << ", update " << 1 / (TIME_COUNT - t) << " fps" << endl;
    }

    pthread_exit(NULL);
}

void camera::start()
{
    if (!thread_run_ && image_capture_pair_) {
        int ret = pthread_create(&pthread_, NULL, threadCaptureCamera, 
                                 (void *)(image_capture_pair_));
        if (ret != 0) {
            printf("pthread_create error: error_code = %d\n", ret);
            exit(-1);
        }
        thread_run_ = true;
    }
}

void camera::stop()
{
    if (thread_run_) {
        pthread_cancel(pthread_);
        thread_run_ = false;
    }
}

void camera::originImage(cv::Mat & dst)
{
    dst = image_capture_pair_->first.clone();
}

void camera::undistortImage(cv::Mat & dst)
{
    cv::Mat src = image_capture_pair_->first.clone();
    if (undistort_) {
        doUndistort(src, dst);
    } else {
        dst = src.clone();
    }
}

cv::String camera::id()
{
    if (camera_type_ == RTSP) {
        return ip_;
    } else {
        return to_string(addr_id_);
    }
}

} //CC


void initCameraParams(std::vector<CC::camera> & cap_vector)
{
    /* set user passwd */
    cv::String user = "admin";
    cv::String passwd = "aaron20127";

    /* set camera intrinsic */
    double intrinsic[][9] = {
    2.6182e+03, 0, 973.1464, 0, 2.6228e+03, 571.7691, 0, 0, 1,
    2.6182e+03, 0, 973.1464, 0, 2.6228e+03, 571.7691, 0, 0, 1,
    };

    /* set camera undistort */
    double distortion[][5] = {
    -0.5222, -0.2738, 0, 0, 0,
    -0.5222, -0.2738, 0, 0, 0,
    };     

    /* set camera ip */
    char * camera_ip[] = {
    "192.168.0.111", 
    "192.168.0.112"
    };

    /* RSTP */
    int size_camera = sizeof(camera_ip) / sizeof(char *);
    for (int i = 0; i < size_camera; i++) {
        cv::String ip(camera_ip[i]);
        CC::camera camera(user, passwd, ip, intrinsic[i], distortion[i]);
        cap_vector.push_back(camera);
    }

    /* USB */
    int id = 0;
    CC::camera camera(id);
    cap_vector.push_back(camera);

}


int main()
{
    /* */
    // debug::DEBUG("dsfafdfdasf");
    // while(true) {
    //     sleep(10);
    // }

    std::vector<CC::camera> cap;

    /* init */
    initCameraParams(cap);

    std::cout << "start..ok." << std::endl;

    for (int i = 0; i < cap.size(); i++) {
        cap[i].start();
        cv::namedWindow(cap[i].id(),0);    
    } 

    /* show */
    cv::Mat dst;
    while(true) {
        for (int i = 0; i < cap.size(); i++) {
            cap[i].undistortImage(dst);
            cv::imshow(cap[i].id(), dst);        
        } 
        cv::waitKey(20);
    }
}