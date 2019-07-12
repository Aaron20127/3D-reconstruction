// Example 19-3. Stereo calibration, rectification, and correspondence
#pragma warning(disable : 4996)
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "misc.h"

using namespace std;

static void help() { 
	std::cout 	<< "\n<Usage>"
			<< "\n>> ./example 'data/left/*.jpg' 'data/right/*.jpg' 9 6 (default)\n"
			<< "\n>> ./example 'data/left/*.jpg' 'data/right/*.jpg' 9 6 \n"
            << "\n argv[1]: left  camera image folder or pattern"
            << "\n argv[2]: right camera image folder or pattern"
            << "\n argv[3]: corners width"
            << "\n argv[4]: corners heigt"
			<< "\n" 
			<< endl;
}

void formatPrintMat(cv::Mat mat, int width=26, int precision=17)
{      
    for(int i=0; i<mat.size().height; i++) {
        for(int j=0; j<mat.size().width; j++) {
            std::cout.precision(precision);
            std::cout.width(width);
            cout << std::left << mat.at<double>(i,j);
            if(j != mat.size().width-1)
                std::cout << "";
            else
                std::cout << endl; 
        }
    }
}

int main(int argc, char **argv) {
    /* 1. initial */
    int board_w = 9; 
    int board_h = 6;
    cv::String left_image_pattern  = "binocular_camera_calibration/data/left/*.jpg";
    cv::String right_image_pattern = "binocular_camera_calibration/data/right/*.jpg";

    if (argc == 1) {
    } else if (argc == 5) {
        cv::String left_image_pattern  = argv[1];
        cv::String right_image_pattern = argv[2];
        board_w = atoi(argv[3]);
        board_h = atoi(argv[4]);
    } else {
        help();
        return 1;
    }

    int board_n = board_w * board_h; // number of corners
    cv::Size board_sz = cv::Size(board_w, board_h); // size of chessboard 


    /* 2. get images from path */
    vector<cv::Mat> images[2];
    misc::readStereoImages(left_image_pattern, right_image_pattern, images[0], images[1]);

    /* 3. find corners, get image points and world points of object */
    vector<vector<cv::Point2f>> image_points[2];
    vector<vector<cv::Point3f>> object_points;
    vector<cv::Mat> drawChessboardCornersImages[2];

    for ( int i=0; i < images[0].size(); i++ ) {
        // find corners
        vector<cv::Point2f> left_corners;
        vector<cv::Point2f> right_corners;

        // cv::imshow("images[0][i]", images[0][i]);
        // cv::imshow("images[1][i]", images[1][i]);
        // cv::waitKey(0);

        bool left_found = cv::findChessboardCorners(images[0][i], board_sz, left_corners); 
        bool right_found = cv::findChessboardCorners(images[1][i], board_sz, right_corners); 

        if (left_found && right_found) {
            // draw coners on image
            cv::Mat left_image;
            cv::Mat right_image;
            cv::drawChessboardCorners(left_image, board_sz, left_corners, left_found);
            cv::drawChessboardCorners(right_image, board_sz, right_corners, right_found);
            drawChessboardCornersImages[0].push_back(left_image);
            drawChessboardCornersImages[1].push_back(right_image);

            // add points
            image_points[0].push_back(left_corners);
            image_points[1].push_back(right_corners);
            object_points.push_back(vector<cv::Point3f>());
            vector<cv::Point3f> &opts = object_points.back();
            opts.resize(board_n);
            for (int j = 0; j < board_n; j++) {
                opts[j] = cv::Point3f((float)(j / board_w), (float)(j % board_w), 0.f);
            }
        }
    }


    /* 4. caliration stereo cameras */
    cv::Size image_size = images[0][0].size();
    cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1, D2, R, T, E, F;
    std::cout << "\nRunning stereo calibration ...\n";
    cv::stereoCalibrate(
                    object_points,
                    image_points[0],
                    image_points[1],
                    M1, D1, M2, D2,
                    image_size,
                    R, T, E, F,
                    cv::CALIB_FIX_ASPECT_RATIO | 
                    cv::CALIB_ZERO_TANGENT_DIST | 
                    cv::CALIB_SAME_FOCAL_LENGTH,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5));
   
    // std::cout << std::fixed << std::cout.precision(10);
    cout << "M1:\n";
    misc::formatPrintMat(M1);
    cout << "\nM2:\n";
    misc::formatPrintMat(M2);
    cout << "\nR:\n";
    misc::formatPrintMat(R);
    cout << "\nT:\n";
    misc::formatPrintMat(T);

    // cv::FileStorage fs("binocular_camera_calibration.json", cv::FileStorage::WRITE);
    // fs << "M1" << M1 << "D1" << D1;
    // fs << "M2" << M2 << "D2" << D2;
    // fs << "R" << R << "T" << T << "E" << E << "F" << F;
    // fs.release();   
    cout << "Done! \n\n";




    /* 5. calibration quality check */
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    vector<cv::Point3f> lines[2];
    double avgErr = 0;
    int nframes = (int)object_points.size();
    for (int i = 0; i < nframes; i++) {
        vector<cv::Point2f> &pt0 = image_points[0][i];
        vector<cv::Point2f> &pt1 = image_points[1][i];
        cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);
        cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);
        cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
        cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);

        for (int j = 0; j < board_n; j++) {
        double err = fabs(pt0[j].x * lines[1][j].x + pt0[j].y * lines[1][j].y +
                            lines[1][j].z) +
                    fabs(pt1[j].x * lines[0][j].x + pt1[j].y * lines[0][j].y +
                            lines[0][j].z);
        avgErr += err;
        }
    }
    std::cout << "avg err = " << avgErr / (nframes * board_n) << endl;


    /* 6. compute and display rectification */
    bool showUndistorted = true;
    bool useUncalibrated = true;
    bool isVerticalStereo = false; // horiz or vert cams

    if (showUndistorted) {
        cv::Mat R1, R2, P1, P2, map11, map12, map21, map22;

        // IF BY CALIBRATED (BOUGUET'S METHOD)
        //
        if (!useUncalibrated) {
            stereoRectify(M1, D1, M2, D2, image_size, R, T, R1, R2, P1, P2,
                        cv::noArray(), 0);
        isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
        // Precompute maps for cvRemap()
        initUndistortRectifyMap(M1, D1, R1, P1, image_size, CV_16SC2, map11,
                                map12);
        initUndistortRectifyMap(M2, D2, R2, P2, image_size, CV_16SC2, map21,
                                map22);
        }
        // OR ELSE HARTLEY'S METHOD
        else {
            // use intrinsic parameters of each camera, but
            // compute the rectification transformation directly
            // from the fundamental matrix
            vector<cv::Point2f> allpoints[2];
            for (int i = 0; i < nframes; i++) {
                copy(image_points[0][i].begin(), image_points[0][i].end(),
                    back_inserter(allpoints[0]));
                copy(image_points[1][i].begin(), image_points[1][i].end(),
                    back_inserter(allpoints[1]));
            }
            cv::Mat F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);
            cv::Mat H1, H2;
            cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, image_size,
                                            H1, H2, 3);
            R1 = M1.inv() * H1 * M1;
            R2 = M2.inv() * H2 * M2;

            // Precompute map for cvRemap()
            //
            cv::initUndistortRectifyMap(M1, D1, R1, P1, image_size, CV_16SC2, map11,
                                        map12);
            cv::initUndistortRectifyMap(M2, D2, R2, P2, image_size, CV_16SC2, map21,
                                        map22);
        }



        /* 7.rectify the images and find disparity maps */
        cv::Mat pair;
        if (!isVerticalStereo)
            pair.create(image_size.height, image_size.width * 2, CV_8UC3);
        else
            pair.create(image_size.height * 2, image_size.width, CV_8UC3);

        // Setup for finding stereo corrrespondences
        cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
            -64, 128, 11, 100, 1000, 32, 0, 15, 1000, 16, cv::StereoSGBM::MODE_HH);

        for (int i = 0; i < nframes; i++) {
            cv::Mat img1;
            cv::Mat img2;
            cv::cvtColor(images[0][i], img1, cv::COLOR_BGR2GRAY);
            cv::cvtColor(images[1][i], img2, cv::COLOR_BGR2GRAY);

            cv::Mat img1r, img2r, disp, vdisp;
            if (img1.empty() || img2.empty())
                continue;
            cv::remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
            cv::remap(img2, img2r, map21, map22, cv::INTER_LINEAR);
            if (!isVerticalStereo || !useUncalibrated) {

                // When the stereo camera is oriented vertically,
                // Hartley method does not transpose the
                // image, so the epipolar lines in the rectified
                // images are vertical. Stereo correspondence
                // function does not support such a case.
                stereo->compute(img1r, img2r, disp);
                cv::normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
                cv::imshow("disparity", vdisp);
            }
            if (!isVerticalStereo) {
                cv::Mat part = pair.colRange(0, image_size.width);
                cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
                part = pair.colRange(image_size.width, image_size.width * 2);
                cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
                for (int j = 0; j < image_size.height; j += 16)
                cv::line(pair, cv::Point(0, j), cv::Point(image_size.width * 2, j),
                        cv::Scalar(0, 255, 0));
            } else {
                cv::Mat part = pair.rowRange(0, image_size.height);
                cv::cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
                part = pair.rowRange(image_size.height, image_size.height * 2);
                cv::cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
                for (int j = 0; j < image_size.width; j += 16)
                line(pair, cv::Point(j, 0), cv::Point(j, image_size.height * 2),
                    cv::Scalar(0, 255, 0));
            }
            cv::imshow("rectified", pair);
            if ((cv::waitKey() & 255) == 27)
                break;
        }
    }

    return 0;
}
