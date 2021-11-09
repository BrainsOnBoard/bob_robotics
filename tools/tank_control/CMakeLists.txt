cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(tank_control)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid net os robots
             robots/tank/net video)

add_executable(tank_control tank_control.cc)
target_link_libraries(tank_control PUBLIC ${BoBRobotics_LIBRARIES})
