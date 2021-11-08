cmake_minimum_required(VERSION 3.10)
project(robot_positioner_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid robots robots/control vicon)

add_executable(robot_positioner robot_positioner.cc)
target_link_libraries(robot_positioner PUBLIC ${BoBRobotics_LIBRARIES})
