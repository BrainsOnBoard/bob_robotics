cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES norbot.cc tank.cc tank_netsink.cc
           BOB_MODULES common net hid)
