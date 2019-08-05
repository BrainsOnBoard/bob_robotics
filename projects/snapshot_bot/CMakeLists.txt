cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

# Default to building snapshot_bot
if(NOT TARGET)
    set(TARGET snapshot_bot)
endif()

if(${TARGET} STREQUAL snapshot_bot)
    BoB_project(EXECUTABLE snapshot_bot
                SOURCES snapshot_bot.cc memory.cc image_input.cc
                BOB_MODULES antworld common hid imgproc navigation net vicon video)
else()
    BoB_project(EXECUTABLE offline_train
                SOURCES offline_train.cc memory.cc image_input.cc
                BOB_MODULES antworld common imgproc navigation video)
endif()
