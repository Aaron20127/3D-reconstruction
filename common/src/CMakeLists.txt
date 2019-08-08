cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

# ${PROJECT_SOURCE_DIR} represents the current directory of CmakeLists.txt
include_directories("${PROJECT_SOURCE_DIR}/../include")


add_library(
    common SHARED 
    misc.cpp
    imageUndistort.cpp
    imageUndistort.cpp
    camera.cpp debug.cpp
)


