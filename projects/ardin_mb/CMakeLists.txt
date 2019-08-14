cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(EXECUTABLE ardin_mb
            SOURCES ardin_mb.cc state_handler.cc mb_memory.cc opencv_texture.cc mb_memory_ardin.cc mb_memory_hog.cc vector_field.cc visual_navigation_ui.cc imgui_impl_sfml.cpp
            BOB_MODULES common antworld navigation
            THIRD_PARTY imgui)

# Kludge to stop Logger::logger being defined multiple times
add_definitions(-DNO_HEADER_DEFINITIONS)

BoB_add_link_libraries("dl")
