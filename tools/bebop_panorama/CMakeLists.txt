cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES bebop_panorama.cc
            BOB_MODULES common hid robots/bebop)
