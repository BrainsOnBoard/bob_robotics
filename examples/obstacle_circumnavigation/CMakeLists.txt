cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES simulated_obstacle_avoiding_positioner.cc
                    simulated_obstacle_circumnavigation.cc
                    vicon_obstacle_avoiding_positioner.cc
                    vicon_obstacle_circumnavigation.cc
            BOB_MODULES common hid navigation net robots robots/control vicon
                        viz/sfml_world)
