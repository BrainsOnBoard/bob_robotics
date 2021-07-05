cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES infomax_matlab_comparison.cc infomax_route_example.cc
            BOB_MODULES common navigation
            THIRD_PARTY matplotlibcpp)
