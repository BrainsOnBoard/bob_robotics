cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)

if(NO_ODK2)
    add_definitions(-DNO_ODK2)
    BoB_project(SOURCES display.cc unwrap_display.cc
                BOB_MODULES common video)
else()
    BoB_project(SOURCES display.cc unwrap_display.cc
                BOB_MODULES common video video/odk2)
endif()
