cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES betaflight_test.cc
            BOB_MODULES common hid robots video
            PLATFORMS unix
            THIRD_PARTY msp)
