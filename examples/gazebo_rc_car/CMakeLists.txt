cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gazebo_rc_car_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common video robots robots/gazebo)
find_package(Gazebo REQUIRED)

add_executable(gazebo_rc_car gazebo_ackermann.cc)
target_link_libraries(gazebo_rc_car PUBLIC ${BoBRobotics_LIBRARIES})
add_dependencies(gazebo_rc_car BoBRobotics::gazebo_rc_car_plugin)


