cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES lm9ds1.cc
            BOB_MODULES common
            PLATFORMS linux)
