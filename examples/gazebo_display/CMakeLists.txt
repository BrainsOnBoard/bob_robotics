cmake_minimum_required(VERSION 3.10)
project(gazebo_display_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED video robots/gazebo)
find_package(Gazebo REQUIRED)

add_executable(gazebo_display gazebo_display.cc)
target_link_libraries(gazebo_display ${BoBRobotics_LIBRARIES})
