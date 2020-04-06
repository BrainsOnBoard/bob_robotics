cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES opengl.cc
           BOB_MODULES video
           EXTERNAL_LIBS glew)
