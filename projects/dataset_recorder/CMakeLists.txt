cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(IS_EXPERIMENT TRUE
            SOURCES dataset_recorder.cc
            BOB_MODULES common imgproc navigation robots video)
