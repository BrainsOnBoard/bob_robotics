cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES read_example.cc write_example.cc
            BOB_MODULES common navigation video)
