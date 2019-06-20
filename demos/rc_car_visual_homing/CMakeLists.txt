cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES test.cc train.cc
            BOB_MODULES common imgproc hid navigation robots video)
