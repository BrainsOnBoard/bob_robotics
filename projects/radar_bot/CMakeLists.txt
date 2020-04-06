cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

BoB_project(EXECUTABLE simulator
            SOURCES simulator.cc
            BOB_MODULES common hid robots
            GENN_MODEL model.cc
            GENN_CPU_ONLY TRUE
            EXTERNAL_LIBS serial)
