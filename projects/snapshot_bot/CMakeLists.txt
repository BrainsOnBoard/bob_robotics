cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

# Default to building snapshot_bot
if(NOT TARGET)
    set(TARGET snapshot_bot)
endif()



if(${TARGET} STREQUAL snapshot_bot)
    if(USE_ODK2)
        add_definitions(-DUSE_ODK2)
        BoB_project(IS_EXPERIMENT TRUE
                EXECUTABLE snapshot_bot
                SOURCES snapshot_bot.cc memory.cc image_input.cc
                BOB_MODULES common hid imgproc navigation robots robots/control vicon video video/odk2)
    else()
        BoB_project(IS_EXPERIMENT TRUE
                EXECUTABLE snapshot_bot
                SOURCES snapshot_bot.cc memory.cc image_input.cc
                BOB_MODULES common hid imgproc navigation robots robots/control vicon video)
    endif()

else()
    BoB_project(IS_EXPERIMENT TRUE
                EXECUTABLE offline_train
                SOURCES offline_train.cc memory.cc image_input.cc
                BOB_MODULES common imgproc navigation video)
endif()
