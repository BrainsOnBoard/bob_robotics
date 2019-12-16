cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES save_object.cc
            BOB_MODULES common
            THIRD_PARTY matplotlibcpp vicon_data_stream_sdk
            EXTERNAL_LIBS opencv)
