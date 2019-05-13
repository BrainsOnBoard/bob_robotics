cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES ev3_control.cc
            BOB_MODULES net robots robots/ev3 video
            PLATFORMS linux)
