cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES simulated_ackermann_car.cc
            BOB_MODULES robots viz/car_display)
