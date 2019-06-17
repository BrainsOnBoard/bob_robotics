cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES net.cc screen.cc video.cc
           BOB_MODULES common)
