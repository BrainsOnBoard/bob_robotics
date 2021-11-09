cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(snapshot_bot_remote_display)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation net video)

add_executable(snapshot_bot_remote_display snapshot_bot_remote_display.cc)
target_link_libraries(snapshot_bot_remote_display PUBLIC ${BoBRobotics_LIBRARIES})
