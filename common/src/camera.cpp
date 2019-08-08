
#include "camera.h"

namespace camera {

camera::camera () 
{
    pthread_ = 0;
    thread_run_ = false;
    image_capture_pair_ = NULL;
};

camera::camera(const camera & st) 
    : imageUndistort(st)
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
}

// assignment
camera & camera::operator=(const camera &st)
{
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

camera::~camera()
{
    stop();
    delete image_capture_pair_;
}


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
    image_capture_pair_ = 
        new ImageVediocapturePairType (image.clone(), capture_);
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
    image_capture_pair_ = 
        new ImageVediocapturePairType (image.clone(), capture_);
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
    // double t;
    while(true) {
        // t = timeStamp();
        cap.grab();
        cap.retrieve(dst);

        // cout << "pt " <<  params << ", update " 
        //      << 1 / timeStampDiff(t) << " fps" << endl;
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

} //camera