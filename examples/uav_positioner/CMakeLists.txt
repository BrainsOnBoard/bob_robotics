cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES uav_positioner.cc
            BOB_MODULES robots/bebop robots/control vicon)