cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES odk2.cc
           BOB_MODULES video
           THIRD_PARTY development_kit_odk2)
