cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_module(SOURCES odk2.cc
           THIRD_PARTY development_kit_odk2
           EXTERNAL_LIBS eigen3)
