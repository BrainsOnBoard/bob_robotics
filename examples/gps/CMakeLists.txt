cmake_minimum_required(VERSION 3.10)
project(gps_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS gps)

add_executable(gps gps.cc)
target_link_libraries(gps PUBLIC BoBRobotics::gps)
