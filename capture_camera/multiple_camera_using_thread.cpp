
#include "camera.h"
#include "timing.h"
#include <sys/time.h>

void initCameraParams(std::vector<camera::camera> & cap_vector)
{
    /* RSTP */
    /* set user passwd */
    cv::String user = "admin";
    cv::String passwd = "aaron20127";

    /* set camera ip */
    char * camera_ip[] = {
    "192.168.0.111", 
    "192.168.0.112"
    };

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

    int size_camera = sizeof(camera_ip) / sizeof(char *);
    for (int i = 0; i < size_camera; i++) {
        cv::String ip(camera_ip[i]);
        camera::camera camera(user, passwd, ip, intrinsic[i], distortion[i]);
        cap_vector.push_back(camera);
    }

    /* USB */
    int id = 0;
    camera::camera camera(id);
    cap_vector.push_back(camera);
}


int main(int argc, char *argv[])
{
    // double t = timing::timeStamp();
    // double diff = timing::timeStampDiff(t);
    // std::cout << "timediff: " << diff << std::endl;

    // sleep(100);
    std::vector<camera::camera> cap;

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