cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES bebop_indoor_route_db_creator.cc
            BOB_MODULES common hid navigation robots/bebop vicon)
