#include "opencv2/opencv.hpp"
#include <iostream>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <unistd.h>
#include <chrono>


using namespace std;


void captureSingleCameraAndSend(cv::String &user, 
                                cv::String &passwd,
                                cv::String &ip,
                                int & total_image_size,
                                int & fd_read,
                                int & fd_write)
{
    /* no block to read */
    int flags = fcntl(fd_read, F_GETFL);
    fcntl(fd_read,F_SETFL,flags | O_NONBLOCK);

    cv::String url=cv::String("rtsp://") + user + ':' + passwd + "@" + ip +
                            "//Streaming/Channels/1";
    cv::VideoCapture cap = cv::VideoCapture(url);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        exit(0);
    }

    cout << "cap open ok!" << endl;

    /* Continuous capture */
    cv::Mat dst;
    int ret = 0;
    int size = total_image_size;
    int count = 0;
    char buff[2] = {0};

    while(true) {
        /* get send cmd */
        ret = read(fd_read, buff, 1);
        cout  << "child cmd read: " << ret << ", read fd " << fd_read << endl;

        /* grab image, not decode */
        cap.grab();

        /* if cmd length equal 1, send image*/
        while(ret == 1) {
            /* decode image */
            cap.retrieve(dst);   

            if (dst.empty()) {
                cerr << "ERROR! blank frame grabbed, capture again!\n";
                cap.grab();
            } else {
                cout << "start write image ..." << endl;
                /* send image and waite to receive. need block, 
                   because pipe can only save 65535 byte once */
                ret = write(fd_write, dst.data, size); 
                count ++;
                cout << "child [" << count << "]: write image ok, " 
                     << "write ret " << ret << endl;
            }
        } 
    }
}

int main(int argc, char *argv[])
{
    /* init pipe */
    int fd_process_1[2][2];                  
    int fd_process_2[2][2];               

    if (pipe(fd_process_1[0]) < 0)                  
        cout << "Create Pipe Error, errno " << errno << endl;
    if (pipe(fd_process_1[1]) < 0)                   
        cout << "Create Pipe Error, errno " << errno << endl;
    if (pipe(fd_process_2[0]) < 0)                   
        cout << "Create Pipe Error, errno " << errno << endl;
    if (pipe(fd_process_2[1]) < 0)                  
        cout << "Create Pipe Error, errno " << errno << endl;

    /* create 3 proccess*/
    pid_t pid;
    char buff[6220800];

    if ((pid = fork()) < 0) {           
        printf("Fork Error!\n");
    } else if(pid == 0) {               

        if ((pid = fork()) < 0) {       
             printf("Fork Error!\n");
        } else if(pid == 0) {    
            cv::String user = "admin";
            cv::String passwd = "aaron20127";
            cv::String ip = "192.168.0.111";
            int total_image_size = 1920*1080*3;

            captureSingleCameraAndSend(user, passwd, ip,
                total_image_size, fd_process_1[0][0], fd_process_1[1][1]);
        } else {

            cv::String user = "admin";
            cv::String passwd = "aaron20127";
            cv::String ip = "192.168.0.112";
            int total_image_size = 1920*1080*3;

            captureSingleCameraAndSend(user, passwd, ip,
                total_image_size, fd_process_2[0][0], fd_process_2[1][1]);
        }
    } else {
        cout << "read start!" << endl;

        cv::namedWindow("image_read", 0);
        cv::namedWindow("image_read1", 0);
        /* remap matrix to buff */
        cv::Mat image_read(1080, 1920, CV_8UC3, buff);       

        int image_size = 1920*1080*3;
        int ret = 0;
        int count = 0;

        /* get current clock count, note chrono just for count not for timestamp */
        #define t_mic  (chrono::duration_cast<chrono::duration<double>>(\
                        chrono::steady_clock::now().time_since_epoch())).count()

        double t1, t2;
        for (;;) {
            count ++;
            cout.precision(15);
            usleep(50000);

            /* send cmd, call first camera to get image */
            t1 = t_mic;
            ret = write(fd_process_1[0][1], "K", 1); 
            cout << "main  [" << count << "]: send cmd to proccess 1 ret " << ret 
                 << ", time " << t1 << endl;

            /* send cmd, call send camera to get image */
            t2 = t_mic;
            ret = write(fd_process_2[0][1], "K", 1); 
            cout << "main  [" << count << "]: send cmd to proccess 2 ret " << ret 
                 << ", time " << t2 <<  endl;

            /* receive image from first camera */
            int read_size = 0;  
            for(;;) {          
                ret = read(fd_process_1[1][0], buff + read_size, image_size - read_size);
                cout << "main  [" << count << "]: receive process 1, image ret " << ret 
                     << ", using " << (t_mic - t1) << endl;

                if (ret > 0) {
                    read_size += ret;
                }

                if (read_size == image_size) {
                    break;
                }
            }
            cv::imshow("image_read", image_read);
            cv::waitKey(1);

            /* receive image from second camera */
            read_size = 0;  
            for(;;) {          
                ret = read(fd_process_2[1][0], buff + read_size, image_size - read_size);
                cout << "main  [" << count << "]: receive process 2, image ret " << ret 
                     << ", using " << (t_mic - t2) << endl;

                if (ret > 0) {
                    read_size += ret;
                }

                if (read_size == image_size) {
                    break;
                }
            }

            cv::imshow("image_read1", image_read);
            cv::waitKey(1);
        }
    }

    return 0;
}