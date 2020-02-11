cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NO_GENN)
    add_definitions(-DNO_GENN)
    BoB_project(EXECUTABLE ardin_mb
                SOURCES ardin_mb.cc state_handler.cc vector_field.cc
                BOB_MODULES common antworld navigation)
else()
    BoB_project(EXECUTABLE ardin_mb
                SOURCES ardin_mb.cc state_handler.cc mb_memory.cc vector_field.cc
                GENN_MODEL model.cc
                BOB_MODULES common antworld navigation)
endif()
