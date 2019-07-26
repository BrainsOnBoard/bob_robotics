cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES pure_pursuit_controller.cc
            BOB_MODULES robots/control viz/car_display)
