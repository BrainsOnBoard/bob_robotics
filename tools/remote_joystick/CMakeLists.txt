cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(remote_joystick)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid net hid/net)

add_executable(remote_joystick remote_joystick.cc)
target_link_libraries(remote_joystick PUBLIC ${BoBRobotics_LIBRARIES})
