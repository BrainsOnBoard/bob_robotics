cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NOT TARGET)
    set(TARGET simulator)
endif()

if(${TARGET} STREQUAL simulator)
    set(SOURCES simulator.cc spline.cc visualizationCommon.cc)
    set(BOB_MODULES common)
elseif(${TARGET} STREQUAL simulatorReplay)
    set(SOURCES simulatorReplay.cc visualizationCommon.cc)
    set(BOB_MODULES common)
elseif(${TARGET} STREQUAL robot)
    set(SOURCES robot.cc visualizationCommon.cc)
    set(BOB_MODULES common hid imgproc net os robots robots/tank video)
else()
    message(FATAL_ERROR "Bad target specified")
endif()

BoB_project(IS_EXPERIMENT TRUE
            EXECUTABLE ${TARGET}
            SOURCES ${SOURCES}
            BOB_MODULES ${BOB_MODULES}
            GENN_MODEL model.cc
            GENN_CPU_ONLY TRUE
            EXTERNAL_LIBS opencv
            OPTIONS RECORD_ELECTROPHYS RECORD_SENSORS USE_SEE3CAM USE_EV3)
