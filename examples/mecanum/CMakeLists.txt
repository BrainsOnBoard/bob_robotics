cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(mecanum_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid robots/omni2d)

add_executable(mecanum mecanum.cc)
target_link_libraries(mecanum PUBLIC ${BoBRobotics_LIBRARIES})
