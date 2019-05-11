cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES camera_recorder.cc
            BOB_MODULES common hid imgproc robots video vicon
            PLATFORMS linux)

if(VICON_UPDATE)
    add_definitions(-DVICON_UPDATE)
endif()
