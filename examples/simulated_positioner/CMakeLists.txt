cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES simulated_positioner.cc
            BOB_MODULES robots robots/control viz/sfml_world)
