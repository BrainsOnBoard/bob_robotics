cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gantry)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/gantry video)
add_executable(gantry gantry.cc)
target_link_libraries(gantry PUBLIC ${BoBRobotics_LIBRARIES})
