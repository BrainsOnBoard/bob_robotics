cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(EXECUTABLE ardin_mb
            SOURCES ardin_mb.cc state_handler.cc mb_memory.cc vector_field.cc
            GENN_MODEL model.cc
            BOB_MODULES antworld common navigation)

# Kludge to stop Logger::logger being defined multiple times
add_compile_definitions(NO_HEADER_DEFINITIONS)
