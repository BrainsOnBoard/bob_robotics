cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(lm9ds1_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common)

add_executable(lm9ds1 lm9ds1.cc)
target_link_libraries(lm9ds1 PUBLIC ${BoBRobotics_LIBRARIES})
