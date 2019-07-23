cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES spineml_braitenberg.cc
            BOB_MODULES common hid navigation robots robots/control viz/sfml_world
            EXTERNAL_LIBS spineml_simulation)
