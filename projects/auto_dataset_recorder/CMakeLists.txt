cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

set(NAME auto_dataset_recorder)

if(NOT DEFINED SIMULATEDJ)
    set(SIMULATED TRUE)
endif()

if(SIMULATED)
    set(NAME ${NAME}_simulated)
endif()

BoB_project(IS_EXPERIMENT TRUE
            SOURCES ${NAME}.cc
            BOB_MODULES common navigation robots/ackermann robots/control video
            THIRD_PARTY matplotlibcpp)
