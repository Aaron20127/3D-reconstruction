cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
# find_package( OpenCV REQUIRED )

add_subdirectory (monocular_camera_calibration)
add_subdirectory (binocular_camera_calibration)
add_subdirectory (capture_camera)
add_subdirectory (common)
add_subdirectory (mytest)





