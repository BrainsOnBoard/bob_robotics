cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES ant_tank.cc
            BOB_MODULES antworld hid video)
