cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES perfect_memory.cc plot_ridf.cc
            BOB_MODULES common navigation viz)
