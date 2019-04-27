cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES antworld_rotater.cc image_database.cc plot.cc
                   visual_navigation_base.cc
           BOB_MODULES common imgproc
           EXTERNAL_LIBS opencv eigen3
           THIRD_PARTY matplotlibcpp)
