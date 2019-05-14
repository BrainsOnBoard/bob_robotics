cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES simulated_tank_pid.cc
            BOB_MODULES hid robots robots/control viz/sfml_world)
