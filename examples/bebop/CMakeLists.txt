cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES bebop.cc bebop_relative_move.cc
            BOB_MODULES common hid robots/bebop video
            PLATFORMS unix)
