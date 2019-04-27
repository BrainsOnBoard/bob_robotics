cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES sfml_world.cc
           EXTERNAL_LIBS sfml-graphics)
