cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
if(NOT TARGET)
    set(TARGET snapshot_bot)
endif()

message("Building ${TARGET}")
if(${TARGET} STREQUAL snapshot_bot)
    BoB_project(EXECUTABLE ${TARGET}
                SOURCES snapshot_bot.cc memory.cc image_input.cc
                BOB_MODULES common hid imgproc navigation net robots vicon video)
    
    # My compiler is giving warnings about the use of system()
    add_compile_flags(-Wno-unused-result)
elseif(${TARGET} STREQUAL offline_train)
    BoB_project(EXECUTABLE ${TARGET}
                SOURCES offline_train.cc memory.cc image_input.cc
                BOB_MODULES common imgproc navigation video)
else()
    message(FATAL_ERROR "${TARGET} is not a valid target: choose snapshot_bot or offline_train")
endif()
