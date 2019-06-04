cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(EXECUTABLE ardin_mb
            SOURCES ardin_mb.cc state_handler.cc mb_memory.cc vector_field.cc
            GENN_MODEL model.cc
            BOB_MODULES common antworld navigation)

# Kludge to stop Logger::logger being defined multiple times
add_definitions(-DNO_HEADER_DEFINITIONS)
