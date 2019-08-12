cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES gazebo_tank.cc
            GAZEBO_PLUGIN differential_drive_plugin.cc
            BOB_MODULES common hid robots/gazebo video)
