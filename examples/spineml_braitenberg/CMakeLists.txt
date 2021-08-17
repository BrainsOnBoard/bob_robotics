cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES spineml_braitenberg.cc
            BOB_MODULES common hid navigation robots/tank robots/control viz/sfml
            EXTERNAL_LIBS spineml_simulation)
