cmake_minimum_required(VERSION 3.10)
project(opencv_learning)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
add_executable(simple_learning main.cpp)
target_link_libraries( simple_learning ${OpenCV_LIBS} )