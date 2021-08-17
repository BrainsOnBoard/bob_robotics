cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NOT DEFINED TARGET)
    set(TARGET dataset_recorder)
endif()

set(SOURCES ${TARGET}.cc)
set(MODULES common navigation robots/ackermann video)

if(${TARGET} STREQUAL dataset_recorder)
    BoB_project(IS_EXPERIMENT TRUE
                SOURCES ${SOURCES}
                BOB_MODULES ${MODULES} gps imgproc
                OPTIONS DUMMY_CAMERA)
elseif(${TARGET} STREQUAL auto_dataset_recorder_simulated)
    BoB_project(SOURCES ${SOURCES}
                BOB_MODULES ${MODULES} robots/control
                THIRD_PARTY matplotlibcpp)
else()
    message(FATAL_ERROR "${TARGET} is not a valid target")
endif()
