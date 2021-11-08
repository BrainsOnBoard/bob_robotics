cmake_minimum_required(VERSION 3.10)
project(simulated_ackermann_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/ackermann viz/car_display)

add_executable(simulated_ackermann simulated_ackermann.cc)
target_link_libraries(simulated_ackermann PUBLIC ${BoBRobotics_LIBRARIES})
