cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NOT TARGET)
    set(TARGET simulator)
endif()

# Use EV3 robot over the network
if(USE_EV3)
    set(TANK_TYPE BundledTankNetSink)
endif()

if(${TARGET} STREQUAL simulator)
    set(SOURCES simulator.cc visualizationCommon.cc)
    set(BOB_MODULES common)
elseif(${TARGET} STREQUAL simulatorReplay)
    set(SOURCES simulatorReplay.cc visualizationCommon.cc)
    set(BOB_MODULES common)
elseif(${TARGET} STREQUAL robot)
    set(SOURCES robot.cc robotCommon.cc)
    set(BOB_MODULES common hid imgproc robots video)
elseif(${TARGET} STREQUAL robotDeadReckon)
    set(SOURCES robotDeadReckon.cc robotCommon.cc visualizationCommon.cc)
    set(BOB_MODULES common hid imgproc net os robots video)
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
            EXTERNAL_LIBS opencv
            OPTIONS RECORD_ELECTROPHYS RECORD_SENSORS USE_SEE3CAM USE_EV3)
