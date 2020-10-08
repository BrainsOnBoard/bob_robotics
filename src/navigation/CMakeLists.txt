cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES antworld_rotater.cc image_database.cc read_objects.cc
                   visual_navigation_base.cc
           BOB_MODULES common imgproc
           EXTERNAL_LIBS eigen3 opencv openmp)
