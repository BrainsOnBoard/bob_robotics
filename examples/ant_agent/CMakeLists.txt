cmake_minimum_required(VERSION 3.10)
project(ant_agent_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid antworld video)

add_executable(ant_agent ant_agent.cc)
target_link_libraries(ant_agent PUBLIC ${BoBRobotics_LIBRARIES})
