cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES ev3.cc
           BOB_MODULES common robots
           THIRD_PARTY ev3dev-lang-cpp)
