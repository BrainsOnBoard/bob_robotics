cmake_minimum_required(VERSION 3.10)
project(remote_tank)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid net robots/tank/net)

add_executable(remote_tank remote_tank.cc)
target_link_libraries(remote_tank PUBLIC ${BoBRobotics_LIBRARIES})
