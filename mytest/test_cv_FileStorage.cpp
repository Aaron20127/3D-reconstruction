/**
 *  Use cv::FileStorage to store data to file of .xml or .yml
 *  @Note: .yml looks more intuitive
 *  demo address: https://docs.opencv.org/3.4.6/d4/da4/group__core__xml.html
 */


#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    /* 1. write test.yml */
    FileStorage fs("test.yml", FileStorage::WRITE);
    fs << "frameCount" << 5;

    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));

    Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5,1) << 0.1, 0.01, -0.001, 0, 0);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs << "features" << "[";

    for( int i = 0; i < 3; i++ )
    {
        int x = rand() % 640;
        int y = rand() % 480;
        uchar lbp = rand() % 256;
        fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for( int j = 0; j < 8; j++ )
            fs << ((lbp >> j) & 1);
        fs << "]" << "}";
    }
    fs << "]";
    fs.release();

    /* 2. read test.xml*/
    FileStorage fs2("test.yml", FileStorage::READ);
    int frameCount = (int)fs2["frameCount"];

    String date;
    fs2["calibrationDate"] >> date;

    //matrix
    Mat cameraMatrix2, distCoeffs2;
    fs2["cameraMatrix"] >> cameraMatrix2;
    fs2["distCoeffs"] >> distCoeffs2;
    cout << "frameCount: " << frameCount << endl
        << "calibration date: " << date << endl
        << "camera matrix: " << cameraMatrix2 << endl
        << "distortion coeffs: " << distCoeffs2 << endl;

    // list
    FileNode features = fs2["features"];
    for( auto it:features) {
        cout << "feature #: ";
        cout << "x=" << (int)it["x"] << ", y=" << (int)it["y"];

        cout << ", lbp: (";
        for( auto i:(it["lbp"]) ) {
            cout << " " << (int)i;
        }
        cout << ")" << endl;
    }
    fs2.release();

    return 0;
}
