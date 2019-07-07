cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

# include_directories ("${PROJECT_SOURCE_DIR}/monocular_camera_calibration")
# add_subdirectory (monocular_camera_calibration)

# COMPILE EXAMPLES
add_executable( monocular_camera_calibration monocular_camera_calibration.cpp )

# FIRST LINK EXAMPLES:
target_link_libraries( monocular_camera_calibration ${OpenCV_LIBS} )

