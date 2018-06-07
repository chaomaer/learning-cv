cmake_minimum_required(VERSION 3.0)
project(opencv_learning)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
add_executable(stereo_calib_rectify stereo_calib_rectify.cpp)
target_link_libraries(stereo_calib_rectify  ${OpenCV_LIBS} )
