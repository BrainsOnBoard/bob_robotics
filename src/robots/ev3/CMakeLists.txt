cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES ev3.cc mindstorms_imu.cc
           BOB_MODULES common net robots
           THIRD_PARTY ev3dev-lang-cpp)
