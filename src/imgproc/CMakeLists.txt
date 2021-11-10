cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(imgproc)

find_package(BoBRobotics REQUIRED COMPONENTS common)
BoB_module(bee_eye.cc mask.cc opencv_optical_flow.cc opencv_unwrap_360.cc roll.cc)

find_package(OpenCV REQUIRED)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_LIBS})
