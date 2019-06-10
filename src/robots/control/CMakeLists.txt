cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES collision_detector.cc pure_pursuit_controller.cc
                   uav_positioner.cc
           BOB_MODULES common robots vicon
           EXTERNAL_LIBS eigen3 opencv)
