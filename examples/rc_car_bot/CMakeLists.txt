cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES rc_car_bot.cc rc_car_bot_i2c_control.cc
            BOB_MODULES common robots)

