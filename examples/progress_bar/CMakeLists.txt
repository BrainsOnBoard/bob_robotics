cmake_minimum_required(VERSION 3.10)
project(progress_bar_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common)

add_executable(progress_bar progress_bar.cc)
target_link_libraries(progress_bar PUBLIC ${BoBRobotics_LIBRARIES})
