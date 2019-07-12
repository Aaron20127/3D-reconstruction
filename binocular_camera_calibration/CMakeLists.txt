cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

# ${PROJECT_SOURCE_DIR} represents the current directory of CmakeLists.txt
include_directories("${PROJECT_SOURCE_DIR}/../common/include")

# COMPILE EXAMPLES
add_executable( binocular_camera_calibration binocular_camera_calibration.cpp )

# FIRST LINK EXAMPLES:
target_link_libraries( binocular_camera_calibration ${OpenCV_LIBS} misc)
