cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES display.cc unwrap_display.cc
            BOB_MODULES common video)
