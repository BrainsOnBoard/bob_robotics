cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES flat_trim.cc
            BOB_MODULES robots/bebop
            PLATFORMS unix)
