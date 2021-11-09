cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(bebop_calibration)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/uav/bebop)

add_executable(flat_trim flat_trim.cc)
target_link_libraries(flat_trim PUBLIC ${BoBRobotics_LIBRARIES})
