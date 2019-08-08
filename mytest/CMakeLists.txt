cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# ${PROJECT_SOURCE_DIR} represents the current directory of CmakeLists.txt
include_directories("${PROJECT_SOURCE_DIR}/../common/include")

# COMPILE EXAMPLES
add_executable( test_cv_FileStorage test_cv_FileStorage.cpp )
add_executable( test test.cpp )
add_executable( test1 test1.cpp )

# FIRST LINK EXAMPLES:
target_link_libraries( test_cv_FileStorage ${OpenCV_LIBS})
target_link_libraries( test ${OpenCV_LIBS})
target_link_libraries( test1 ${OpenCV_LIBS})

