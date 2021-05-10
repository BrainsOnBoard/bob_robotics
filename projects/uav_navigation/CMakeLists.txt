cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(IS_EXPERIMENT TRUE
            SOURCES uav_navigation.cc
            BOB_MODULES hid navigation robots/bebop
            THIRD_PARTY matplotlibcpp)
