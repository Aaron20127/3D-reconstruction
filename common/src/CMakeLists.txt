cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

add_library(misc SHARED misc.cpp)

