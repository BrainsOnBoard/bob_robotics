cmake_minimum_required(VERSION 3.10)
project(bebop_calibration)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/uav/bebop)

add_executable(flat_trim flat_trim.cc)
target_link_libraries(flat_trim PUBLIC ${BoBRobotics_LIBRARIES})
