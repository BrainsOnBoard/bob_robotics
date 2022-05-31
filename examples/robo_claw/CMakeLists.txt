cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(robo_claw)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common)

add_executable(robo_claw robo_claw.cc)
target_link_libraries(robo_claw PUBLIC BoBRobotics::common)


