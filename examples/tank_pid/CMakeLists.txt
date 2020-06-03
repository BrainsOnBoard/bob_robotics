cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES tank_pid.cc
            BOB_MODULES common hid navigation net robots robots/control vicon
                        viz/sfml
            THIRD_PARTY matplotlibcpp)
