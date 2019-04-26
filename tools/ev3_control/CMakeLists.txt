cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(BOB_MODULES net robots
            THIRD_PARTY ev3dev-lang-cpp
            PLATFORMS linux)
