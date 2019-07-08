/* Form this example, we can learn:
    1. Read images from pattern.
    2. Find chessboard conerns from picture.
    3. Calibrate monocular camera.
    4. Use .xml to save and load data.
    5. Use two images of one camera taken in different.
       postion to calculate binocular fandamental matrix.
    6. Undistort images using camera parameters.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include "misc.h"

using namespace std;

static void help() { 
	cout 	<< "\n<Usage>"
			<< "\n>> ./example 'data/*.jpg' "
			<< "\n>> ./example 'data/*.jpg' 9 6 1.0 \n"
            << "\n argv[1]: image folder or pattern"
            << "\n argv[2]: corners width"
            << "\n argv[3]: corners length"
            << "\n argv[4]: ratio of image resizing"
			<< "\n" 
			<< endl;
}

int main(int argc, char *argv[]) {
  /* 1. initial */
  // Note: As long as the specified width and length are 
  // the maximum number of corners inside the checkerboard,
  // all images can be fully detected. And it needn't resize.
  int board_w = 9;      //chessboard corner width needed to detect
  int board_h = 6;      //chessboard corner length needed to detect
  float image_sf = 1.0; //use to resize the image 0.0-1.0 
  cv::String pattern = "monocular_camera_calibration/data/left*.jpg";
                        // image folder or pattern
  if (argc == 1) {
  } else if (argc == 2) {
      pattern = argv[1];
  } else if (argc == 5) {
      pattern = argv[1];
      board_w = atoi(argv[2]);
      board_h = atoi(argv[3]);
      image_sf = atof(argv[4]);
  } else {
      help();
      return 1;
  }

  int board_n = board_w * board_h;
  cv::Size board_sz = cv::Size(board_w, board_h);


  /* 2. get images from path */
  vector<cv::Mat> images;
  misc::readImagesWithPattern(pattern, images);


  /* 3. find corners, get image points and world points of object */
  vector<vector<cv::Point2f>> image_points;
  vector<vector<cv::Point3f>> object_points;

  for ( auto & image:images ) {
      // find corners
      vector<cv::Point2f> corners;
      cv::resize(image, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);
      bool found = cv::findChessboardCorners(image, board_sz, corners); //BGR image

      if (found) {
          cv::drawChessboardCorners(image, board_sz, corners, found);

          // add points
          image_points.push_back(corners);
          object_points.push_back(vector<cv::Point3f>());
          vector<cv::Point3f> &opts = object_points.back();
          opts.resize(board_n);
          for (int j = 0; j < board_n; j++) {
              opts[j] = cv::Point3f((float)(j / board_w), (float)(j % board_w), 0.f);
          }
      }
  }

  cout << "accept images: " << image_points.size() 
       << " / " << images.size() << endl;


  /* 4. calibrate the camera! */
  cv::Size image_size = images[0].size();
  cv::Mat intrinsic_matrix, distortion_coeffs;
  double err = cv::calibrateCamera(
      object_points,     // Vector of vectors of points
                         // from the calibration pattern
      image_points,      // Vector of vectors of projected
                         // locations (on images)
      image_size,        // Size of images used
      intrinsic_matrix,  // Output camera matrix
      distortion_coeffs, // Output distortion coefficients
      cv::noArray(),     // We'll pass on the rotation vectors...
      cv::noArray(),     // ...and the translation vectors
      cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);
  cout << "reprojection error is " << err;


  /* 5. save the intrinsics and distortions */
  cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
  fs << "image_width" << image_size.width << "image_height" << image_size.height
     << "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
     << distortion_coeffs;
  fs.release();


  /* 6. loading xml */
  fs.open("intrinsics.xml", cv::FileStorage::READ);
  cout << "\nimage width: " << (int)fs["image_width"];
  cout << "\nimage height: " << (int)fs["image_height"];

  cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
  fs["camera_matrix"] >> intrinsic_matrix_loaded;
  fs["distortion_coefficients"] >> distortion_coeffs_loaded;
  cout << "\n\nintrinsic matrix:\n" << intrinsic_matrix_loaded << endl;
  cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl << endl;


  /* 7. Find fundamental mat. But we just use one camera with 
        two pictures which been tacken in different position.
        and corners of two images are match pointing. */
  cv::undistortPoints(
      image_points[0],   // Observed point coordinates (from frame 0)
      image_points[0],   // undistorted coordinates (in this case,
                         // the same array as above)
      intrinsic_matrix,  // Intrinsics, from cv::calibrateCamera()
      distortion_coeffs, // Distortion coefficients, also
                         // from cv::calibrateCamera()
      cv::Mat(),         // Rectification transformation (but
                         // here, we don't need this)
      intrinsic_matrix   // New camera matrix
      );

  cv::undistortPoints(
      image_points[1],   // Observed point coordinates (from frame 1)
      image_points[1],   // undistorted coordinates (in this case,
                         // the same array as above)
      intrinsic_matrix,  // Intrinsics, from cv::calibrateCamera()
      distortion_coeffs, // Distortion coefficients, also
                         // from cv::calibrateCamera()
      cv::Mat(),         // Rectification transformation (but
                         // here, we don't need this)
      intrinsic_matrix   // New camera matrix
      );

  // Since all the found chessboard corners are inliers, i.e., they
  // must satisfy epipolar constraints, here we are using the
  // fastest, and the most accurate (in this case) 8-point algorithm.
  cv::Mat F = cv::findFundamentalMat( // Return computed matrix
      image_points[0],                // Points from frame 0
      image_points[1],                // Points from frame 1
      cv::FM_8POINT                   // Use the 8-point algorithm
      );
  cout << "Fundamental matrix: \n" << F << endl;


  /* 8.undistort images */
  cv::Mat new_camera_intrinsic_matrix = 
  cv::getOptimalNewCameraMatrix (
      intrinsic_matrix,         // intrinsic matrix
      distortion_coeffs,        // distortion coefficients 
      image_size,               // Original image size
      1.0                       // 1.0 retain all black edge,
                                // 0.0 cut off all the black edges.
      );

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap(
      intrinsic_matrix,  // Our camera matrix
      distortion_coeffs, // Our distortion coefficients
      cv::Mat(),                // (Optional) Rectification, don't
                                // need.
      new_camera_intrinsic_matrix,  // "New" matrix, here it's the same
                                // as the first argument.
      image_size,               // Size of undistorted image we want
      CV_16SC2,                 // Specifies the format of map to use
      map1,                     // Integerized coordinates
      map2                      // Fixed-point offsets for
                                // elements of map1
      );

  // remap and show image
  int index = 0;
  for (auto image:images) {
    cv::Mat image_undistort;
    index++;

    if (image.empty())
      break;
    cv::remap(image, // Input image
              image_undistort,  // Output image
              map1,   // Integer part of map
              map2,   // Fixed point part of map
              cv::INTER_LINEAR, cv::BORDER_CONSTANT,
              cv::Scalar() // Set border values to black
              );
    cv::imshow(to_string(index), image_undistort);
  }
  cv::waitKey(0);

  return 0;
}
