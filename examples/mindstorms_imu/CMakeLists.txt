cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES mindstorms_imu.cc
            BOB_MODULES robots/ev3
            PLATFORMS linux)
