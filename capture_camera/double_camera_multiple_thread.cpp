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

using namespace std;

/**
 *  Grab camera image. It averages 25 frames per second to update.
 */

/* time count (seconds) */
#define TIME_COUNT (chrono::duration_cast<chrono::duration<double>>(\
                    chrono::steady_clock::now().time_since_epoch())).count()

/* pass in parameters to the thread */
typedef struct threadParams
{
    cv::Mat image;
    cv::String ip;
}TypeIpImage;

/* get the RSTP protocol interface */
void getRSTPCapture(cv::String & ip, cv::VideoCapture & cap)
{
    cv::String user = "admin";
    cv::String passwd = "aaron20127";

    cv::String url=cv::String("rtsp://") + user + ':' + passwd + "@" + ip +
                              "//Streaming/Channels/1";
    cap = cv::VideoCapture(url);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        exit(0);
    }
}

/* thread two capture camera */
void *threadCaptureCamera(void * ip_image)
{
    TypeIpImage * p_ip_image = (TypeIpImage * )ip_image;
    cv::Mat dst = p_ip_image->image;

    cv::VideoCapture cap;
    getRSTPCapture(p_ip_image->ip, cap);

    /* Continuous capture */
    double t = TIME_COUNT;
    while(true) {
        t = TIME_COUNT;
        cap.grab();
        cap.retrieve(dst);
        cout << "update " << 1 / (TIME_COUNT - t) << " fps" << endl;
    }

    pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
    /* parameters */
    vector<cv::String> ip = {"192.168.0.111", "192.168.0.112"}; 
    cv::Size image_size(1920,1080);

    /* init */
    int num_threads = ip.size();
    TypeIpImage ip_image[num_threads];
    pthread_t threads[num_threads];

    for (int i = 0; i < num_threads; i++) {
        cv::Mat image(image_size, CV_8UC3);
        ip_image[i].image = image;
        ip_image[i].ip = ip[i];

        int ret = pthread_create(&threads[i], NULL, threadCaptureCamera, (void*)&(ip_image[i]));
        if (ret != 0) {
            printf("pthread_create error: error_code = %d\n", ret);
            exit(-1);
        }
    }
     

    /* show */
    cv::namedWindow(ip[0], 0);
    cv::namedWindow(ip[1], 0);
    while(true) {
        cv::imshow(ip[0], ip_image[0].image);
        cv::imshow(ip[1], ip_image[1].image);
        cv::waitKey(20);
    }

    pthread_exit(NULL);
}
