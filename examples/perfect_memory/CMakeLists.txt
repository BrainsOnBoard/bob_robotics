cmake_minimum_required(VERSION 3.10)
project(perfect_memory_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation viz)

add_executable(perfect_memory perfect_memory.cc)
target_link_libraries(perfect_memory PUBLIC BoBRobotics::navigation)

add_executable(plot_ridf plot_ridf.cc)
target_link_libraries(plot_ridf PUBLIC ${BoBRobotics_LIBRARIES})
