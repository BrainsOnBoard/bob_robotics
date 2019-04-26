cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(BOB_MODULES common imgproc
           EXTERNAL_LIBS opencv eigen3
           THIRD_PARTY matplotlibcpp)
