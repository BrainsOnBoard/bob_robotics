cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(obj_process)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common)

add_executable(obj_process obj_process.cc)
target_link_libraries(obj_process PUBLIC ${BoBRobotics_LIBRARIES})
