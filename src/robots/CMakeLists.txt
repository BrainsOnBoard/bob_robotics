cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES ackermann.cc norbot.cc simulated_ackermann.cc surveyor.cc
                   tank_netsink.cc tank.cc uav.cc
           BOB_MODULES common hid)
