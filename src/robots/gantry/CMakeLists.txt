cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/gantry)

find_package(BoBRobotics REQUIRED)
BoB_module(gantry.cc)

target_link_libraries(${BOB_MODULE_TARGET} PUBLIC "C:\\Program Files\\Advantech\\Motion\\PCI-1240\\Examples\\VC\\LIB\\ADS1240.lib")
target_include_directories(${BOB_MODULE_TARGET} PUBLIC "C:\\Program Files\\Advantech\\Motion\\PCI-1240\\Examples\\Include")
