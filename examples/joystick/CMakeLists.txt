cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid robots)

foreach(TARGET joystick joystick_polling joystick_robot)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC BoBRobotics::hid)
endforeach(TARGET joystick joystick_polling joystick_robot)
target_link_libraries(joystick_robot PUBLIC BoBRobotics::robots)
