cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES joystick.cc joystick_polling.cc joystick_robot.cc
            BOB_MODULES hid robots
            PLATFORMS linux windows)
