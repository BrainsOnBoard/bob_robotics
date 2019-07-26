cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES perfect_memory.cc perfect_memory_antworld.cc plot_ridf.cc
            BOB_MODULES common navigation antworld video hid)
