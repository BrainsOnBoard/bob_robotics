cmake_minimum_required(VERSION 3.10)
project(list_cameras_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common os)

add_executable(list_cameras list_cameras.cc)
target_link_libraries(list_cameras PUBLIC ${BoBRobotics_LIBRARIES})
