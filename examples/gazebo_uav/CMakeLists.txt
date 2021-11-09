cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gazebo_uav_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/gazebo video)
find_package(Gazebo REQUIRED)

add_executable(gazebo_uav gazebo_uav.cc)
target_link_libraries(gazebo_uav PUBLIC ${BoBRobotics_LIBRARIES})
add_dependencies(gazebo_uav BoBRobotics::gazebo_uav_plugin)
