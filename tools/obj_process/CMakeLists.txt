cmake_minimum_required(VERSION 3.10)
project(obj_process)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common)

add_executable(obj_process obj_process.cc)
target_link_libraries(obj_process PUBLIC ${BoBRobotics_LIBRARIES})
