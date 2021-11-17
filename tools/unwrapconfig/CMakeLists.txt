cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(unwrapconfig)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc video)

add_executable(unwrapconfig unwrapconfig.cc)
target_link_libraries(unwrapconfig PUBLIC ${BoBRobotics_LIBRARIES})
