cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(coord_shift_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common)
add_executable(coord_shift coord_shift.cc)
target_link_libraries(coord_shift PUBLIC ${BoBRobotics_LIBRARIES})
