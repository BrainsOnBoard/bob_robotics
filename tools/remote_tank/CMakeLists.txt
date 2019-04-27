cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES remote_tank.cc
            BOB_MODULES common hid net robots
            PLATFORMS linux windows)
