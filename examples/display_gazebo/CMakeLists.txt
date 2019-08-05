cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES display_test.cc
            BOB_MODULES common video
            EXTERNAL_LIBS gazebo)
