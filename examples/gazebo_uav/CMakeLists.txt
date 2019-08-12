cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES gazebo_uav.cc
            GAZEBO_PLUGIN gazebo_quadcopter_plugin.cc
            BOB_MODULES common hid robots/gazebo video)
