cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES generate.cc
            BOB_MODULES common navigation video)

add_definitions(-DNAVIGATION_ALGO=PerfectMemoryRotater<>)
