cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(simulated_tank_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid robots/tank viz/sfml)

add_executable(simulated_tank simulated_tank.cc)
target_link_libraries(simulated_tank PUBLIC ${BoBRobotics_LIBRARIES})
