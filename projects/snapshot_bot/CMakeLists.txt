cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

# Default to building snapshot_bot
if(NOT TARGET)
    set(TARGET snapshot_bot)
endif()

if(${TARGET} STREQUAL snapshot_bot)
    BoB_project(EXECUTABLE snapshot_bot
                SOURCES snapshot_bot.cc memory.cc image_input.cc
                BOB_MODULES common hid imgproc navigation robots vicon video)
elseif(${TARGET} STREQUAL snapshot_bot_visual)
    BoB_project(EXECUTABLE snapshot_bot_visual
                SOURCES snapshot_bot_visual.cc memory.cc image_input.cc
                BOB_MODULES common hid imgproc navigation robots video
                            viz/sfml_world
                OPTIONS LOCAL_DISPLAY) # Run display locally
elseif(${TARGET} STREQUAL offline_train)
    BoB_project(EXECUTABLE offline_train
                SOURCES offline_train.cc memory.cc image_input.cc
                BOB_MODULES common imgproc navigation video)
else()
    message(FATAL_ERROR "Unrecognised target ${TARGET}")
endif()
