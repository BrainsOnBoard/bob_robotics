cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(BOB_MODULES common hid imgproc robots video vicon
            PLATFORMS linux)
