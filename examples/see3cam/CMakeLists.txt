cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES see3cam.cc
            BOB_MODULES common imgproc video
            PLATFORMS linux)
