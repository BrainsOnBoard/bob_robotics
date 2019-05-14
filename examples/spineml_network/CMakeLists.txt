cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES spineml_network.cc
            BOB_MODULES net robots video)
