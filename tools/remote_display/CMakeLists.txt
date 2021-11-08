cmake_minimum_required(VERSION 3.10)
project(remote_display)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS net video)

add_executable(remote_display remote_display.cc)
target_link_libraries(remote_display PUBLIC ${BoBRobotics_LIBRARIES})
