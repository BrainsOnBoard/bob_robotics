cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gazebo_display_example)

find_package(BoBRobotics QUIET REQUIRED video robots/gazebo)
find_package(Gazebo REQUIRED)

add_executable(gazebo_display gazebo_display.cc)
target_link_libraries(gazebo_display ${BoBRobotics_LIBRARIES})
