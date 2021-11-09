cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(bn055_example)

find_package(BoBRobotics QUIET REQUIRED common)

add_executable(bn055 bn055.cc)
target_compile_definitions(bn055 PUBLIC ${BoBRobotics_DEFINITIONS})
target_include_directories(bn055 PUBLIC ${BoBRobotics_INCLUDE_DIRS})
target_link_libraries(bn055 PUBLIC ${BoBRobotics_LIBRARIES})
