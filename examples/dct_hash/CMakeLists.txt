cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(dct_hash_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc)

add_executable(dct_hash dct_hash.cc)
target_link_libraries(dct_hash PUBLIC ${BoBRobotics_LIBRARIES})

configure_file(library1.jpg "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(library2.jpg "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
