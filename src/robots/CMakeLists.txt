cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES norbot.cc surveyor.cc tank_netsink.cc tank.cc uav.cc
           BOB_MODULES common net hid)
