cmake_minimum_required(VERSION 3.10)
project(gazebo_tank_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/gazebo video)
find_package(Gazebo REQUIRED)

add_executable(gazebo_tank gazebo_tank.cc)
target_link_libraries(gazebo_tank PUBLIC ${BoBRobotics_LIBRARIES})
add_dependencies(gazebo_tank BoBRobotics::gazebo_tank_plugin)
