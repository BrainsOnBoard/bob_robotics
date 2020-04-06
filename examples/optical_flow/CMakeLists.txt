cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES optical_flow.cc
            BOB_MODULES common imgproc video)
