cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES tank.cc uav.cc
           BOB_MODULES robots
           EXTERNAL_LIBS gazebo)
