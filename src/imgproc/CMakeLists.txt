cmake_minimum_required(VERSION 2.8)
include(../../make_common/module.cmake)
BoB_build_module(imgproc)

find_package(OpenCV REQUIRED)
target_link_libraries(bob_imgproc PUBLIC ${OpenCV_LIBS})