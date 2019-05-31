cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NOT TARGET)
    set(TARGET simulator)
endif()

if(${TARGET} STREQUAL simulator)
    set(SOURCES simulator.cc visualizationCommon.cc)
elseif(${TARGET} STREQUAL simulatorReplay)
    set(SOURCES simulatorReplay.cc visualizationCommon.cc)
elseif(${TARGET} STREQUAL robot)
    set(SOURCES robot.cc robotCommon.cc)
    set(BOB_MODULES common hid imgproc robots video)
elseif(${TARGET} STREQUAL robotDeadReckon)
    set(SOURCES robotDeadReckon.cc robotCommon.cc)
    set(BOB_MODULES common hid imgproc net robots video)
elseif(${TARGET} STREQUAL robotVicon)
    set(SOURCES robotVicon.cc robotCommon.cc)
    set(BOB_MODULES common hid imgproc robots video vicon)
else()
    message(FATAL_ERROR "Bad target specified")
endif()

BoB_project(EXECUTABLE ${TARGET}
            SOURCES ${SOURCES}
            BOB_MODULES ${BOB_MODULES}
            GENN_MODEL model.cc
            GENN_CPU_ONLY TRUE
            EXTERNAL_LIBS opencv)

if(RECORD_ELECTROPHYS)
    add_definitions(-DRECORD_ELECTROPHYS)
endif()
if(RECORD_SENSORS)
    add_definitions(-DRECORD_SENSORS)
endif()
if(USE_SEE3_CAM)
    add_definitions(-DUSE_SEE3_CAM)
endif()
