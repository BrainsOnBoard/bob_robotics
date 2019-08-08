cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES joystick_linux.cc joystick_windows.cc
           BOB_MODULES common
           PLATFORMS linux windows)
