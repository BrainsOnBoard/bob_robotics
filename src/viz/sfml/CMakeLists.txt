cmake_minimum_required(VERSION 3.10)
project(viz/sfml)

find_package(BoBRobotics REQUIRED COMPONENTS common hid)
BoB_module(joystick_keyboard.cc sfml_world.cc)

find_package(SFML REQUIRED COMPONENTS graphics)
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC sfml-graphics)
