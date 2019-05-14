cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES ant_world_db_creator.cc ant_world_map_creator.cc
            BOB_MODULES common antworld navigation video)
