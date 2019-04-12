cmake_minimum_required(VERSION 2.8)
include(../../make_common/module.cmake)
BoB_build_module(common)

find_library(I2C NAMES i2c)
find_package(OpenCV REQUIRED)
target_link_libraries(bob_common PUBLIC ${OpenCV_LIBS})