cmake_minimum_required(VERSION 3.10)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES generate_infomax.cc generate_pm.cc
            BOB_MODULES common navigation video)
