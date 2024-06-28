cmake_minimum_required(VERSION 3.10)
project(joystick_net)
include(../../cmake/BoBRobotics.cmake)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid hid/net)

add_executable(joystick_net_polling joystick_net_polling.cc)
target_link_libraries(joystick_net_polling PUBLIC ${BoBRobotics_LIBRARIES})
