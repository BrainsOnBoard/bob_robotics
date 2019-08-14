cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES remote_imu.cc
            BOB_MODULES common net robots)
