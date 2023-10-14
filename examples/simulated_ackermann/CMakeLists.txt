cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(simulated_ackermann_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/ackermann viz/sfml)

add_executable(simulated_ackermann simulated_ackermann.cc)
target_link_libraries(simulated_ackermann PUBLIC ${BoBRobotics_LIBRARIES})
