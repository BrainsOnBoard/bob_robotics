cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES rc_car_visual_homing.cc
            BOB_MODULES antworld common hid robots video video/opengl
                        viz/sfml_world
            EXTERNAL_LIBS opencv)
