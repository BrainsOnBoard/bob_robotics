cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES camera.cc tank.cc uav.cc
           BOB_MODULES robots video
           EXTERNAL_LIBS gazebo)
