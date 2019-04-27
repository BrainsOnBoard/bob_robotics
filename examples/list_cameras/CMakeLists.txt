cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES list_cameras.cc
            BOB_MODULES os
            PLATFORMS linux)
