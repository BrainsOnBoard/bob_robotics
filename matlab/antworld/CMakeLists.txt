cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(MEX_SOURCES ant_agent.cc
            EXTERNAL_LIBS matlab)
