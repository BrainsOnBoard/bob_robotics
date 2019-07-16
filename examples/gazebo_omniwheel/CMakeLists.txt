cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(GAZEBO_PLUGIN omniwheel_plugin.cc
            SOURCES gazebo_omniwheel.cc
            BOB_MODULES common hid robots robots/gazebo)
