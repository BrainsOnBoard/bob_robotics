cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE DEBUG)
include(../../cmake/BoBRobotics.cmake)
project(hash_matcher)


find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc)

add_executable(hash_matcher hash_matcher.cc)
target_link_libraries(hash_matcher PUBLIC ${BoBRobotics_LIBRARIES})

