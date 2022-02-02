cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gps_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS gps)

add_executable(gps gps.cc)
target_link_libraries(gps PUBLIC BoBRobotics::gps)
