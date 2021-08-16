cmake_minimum_required(VERSION 3.1)
include(../../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES sink.cc
           BOB_MODULES net robots/tank)
