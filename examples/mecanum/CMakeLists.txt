cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES mecanum.cc mecanum_cam.cc
            BOB_MODULES common hid imgproc video robots)
