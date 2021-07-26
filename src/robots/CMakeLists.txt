cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES ackermann.cc rc_car_bot.cc simulated_ackermann.cc uav.cc
           BOB_MODULES common net hid)
