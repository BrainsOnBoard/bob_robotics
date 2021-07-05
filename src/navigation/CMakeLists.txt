cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES image_database.cc perfect_memory_window.cc read_objects.cc
           BOB_MODULES common imgproc
           EXTERNAL_LIBS eigen3 opencv tbb)
