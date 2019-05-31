cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES robot.cc computer.cc
            BOB_MODULES common hid net os robots video)
