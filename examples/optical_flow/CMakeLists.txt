cmake_minimum_required(VERSION 3.10)
project(optical_flow_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common imgproc video)

add_executable(optical_flow optical_flow.cc)
target_link_libraries(optical_flow PUBLIC ${BoBRobotics_LIBRARIES})
