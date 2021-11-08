cmake_minimum_required(VERSION 3.10)
project(simulated_positioner_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/control robots/tank
             viz/sfml)

add_executable(simulated_positioner simulated_positioner.cc)
target_link_libraries(simulated_positioner PUBLIC ${BoBRobotics_LIBRARIES})
