cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES bee_eye.cc opencv_optical_flow.cc opencv_unwrap_360.cc
           BOB_MODULES common
           EXTERNAL_LIBS opencv)
