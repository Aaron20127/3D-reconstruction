cmake_minimum_required(VERSION 2.8)

project( 3D-reconstrcution )
find_package( OpenCV REQUIRED )

# ${PROJECT_SOURCE_DIR} represents the current directory of CmakeLists.txt
include_directories("${PROJECT_SOURCE_DIR}/../common/include")

# COMPILE EXAMPLES
add_executable( test_cv_FileStorage test_cv_FileStorage.cpp )

# FIRST LINK EXAMPLES:
target_link_libraries( test_cv_FileStorage ${OpenCV_LIBS})

