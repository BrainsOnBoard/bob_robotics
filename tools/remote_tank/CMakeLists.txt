cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(remote_tank)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid net robots/tank/net)

add_executable(remote_tank remote_tank.cc)
target_link_libraries(remote_tank PUBLIC ${BoBRobotics_LIBRARIES})
