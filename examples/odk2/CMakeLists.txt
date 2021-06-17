cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES odk2.cc
            BOB_MODULES common imgproc navigation video/odk2)
