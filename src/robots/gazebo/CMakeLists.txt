cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES node.cc tank.cc uav.cc
           GAZEBO_PLUGINS tank_plugin.cc uav_plugin.cc
           BOB_MODULES robots/tank robots/uav
           EXTERNAL_LIBS gazebo)
