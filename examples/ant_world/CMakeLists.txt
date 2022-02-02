cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(ant_world_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid antworld video viz/sfml)

add_executable(ant_world ant_world.cc)
target_link_libraries(ant_world PUBLIC ${BoBRobotics_LIBRARIES})
