cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES snapshot_bot_remote_display.cc
            BOB_MODULES navigation net video)
