cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES car_display.cc
           EXTERNAL_LIBS sdl2)
