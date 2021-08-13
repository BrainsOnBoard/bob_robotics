cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES joystick_keyboard.cc world.cc
           BOB_MODULES common hid
           EXTERNAL_LIBS sfml-graphics)
