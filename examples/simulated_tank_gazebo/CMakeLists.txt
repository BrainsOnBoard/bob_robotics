cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES simulated_tank_gazebo.cc
            GAZEBO_PLUGIN differential_drive_plugin.cc
            BOB_MODULES common hid robots video)
