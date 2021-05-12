cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NO_GENN)
    add_definitions(-DNO_GENN)
    BoB_project(IS_EXPERIMENT TRUE
                EXECUTABLE ardin_mb
                SOURCES ardin_mb.cc state_handler.cc opencv_texture.cc vector_field.cc visual_navigation_ui.cc
                BOB_MODULES common antworld navigation
                THIRD_PARTY imgui)
else()
    BoB_project(IS_EXPERIMENT TRUE
                EXECUTABLE ardin_mb
                SOURCES ardin_mb.cc state_handler.cc mb_memory.cc opencv_texture.cc mb_memory_ardin.cc vector_field.cc visual_navigation_ui.cc
                BOB_MODULES common antworld navigation
                THIRD_PARTY imgui
                INCLUDE_GENN_USERPROJECTS)
endif()

# Suppress warning from inside imgui
add_compile_flags(-Wno-int-to-pointer-cast)
