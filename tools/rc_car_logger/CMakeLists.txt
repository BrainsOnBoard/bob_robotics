cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES rc_car_logger.cc
            BOB_MODULES common video)
