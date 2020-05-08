cmake_minimum_required(VERSION 3.1)
include(../cmake/bob_robotics.cmake)
BoB_project(SOURCES antworld.cc
            SHARED_LIB antworld
            BOB_MODULES antworld
            EXTERNAL_LIBS python)
