cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES angular_velocity.cc robot.cc speed_test.cc
            BOB_MODULES common hid net robots vicon)
