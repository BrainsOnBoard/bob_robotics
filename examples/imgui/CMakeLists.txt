cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(EXECUTABLE imgui_example
            SOURCES imgui.cc
            BOB_MODULES common
            THIRD_PARTY imgui)
