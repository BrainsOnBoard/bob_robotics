cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(IS_EXPERIMENT TRUE
            SOURCES data_recorder.cc
            BOB_MODULES common robots imgproc video)
