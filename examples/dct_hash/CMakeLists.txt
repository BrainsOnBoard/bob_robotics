cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(dct_hash_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc)

add_executable(dct_hash dct_hash.cc)
target_compile_definitions(dct_hash PUBLIC ${BoBRobotics_DEFINITIONS})
target_include_directories(dct_hash PUBLIC ${BoBRobotics_INCLUDE_DIRS})
target_link_libraries(dct_hash PUBLIC ${BoBRobotics_LIBRARIES})
