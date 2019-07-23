cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES omni2d.cc
           EXTERNAL_LIBS gazebo)
