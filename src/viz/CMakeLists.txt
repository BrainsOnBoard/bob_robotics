cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES plot_ridf.cc
           THIRD_PARTY matplotlibcpp)
