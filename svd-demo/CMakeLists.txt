cmake_minimum_required(VERSION 3.0)
project(opencv_learning)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
add_executable(simple_learning  svd_demo.cpp)
target_link_libraries( simple_learning ${OpenCV_LIBS} )
