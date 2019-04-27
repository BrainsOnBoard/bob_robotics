cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES capture_control.cc udp.cc
           BOB_MODULES common net)
