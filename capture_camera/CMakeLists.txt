cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

# ${PROJECT_SOURCE_DIR} represents the current directory of CmakeLists.txt
include_directories("${PROJECT_SOURCE_DIR}/../common/include")

# COMPILE EXAMPLES
add_executable( double_camera_multiple_proccess_pipe double_camera_multiple_proccess_pipe.cpp )
add_executable( double_camera_multiple_thread double_camera_multiple_thread.cpp )

# FIRST LINK EXAMPLES:
target_link_libraries( double_camera_multiple_proccess_pipe ${OpenCV_LIBS})
target_link_libraries( double_camera_multiple_thread ${OpenCV_LIBS})

