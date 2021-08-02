cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES norbot.cc tank_base.cc
           BOB_MODULES common hid)
