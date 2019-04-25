cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(BOB_MODULES net robots
            THIRD_PARTY ev3dev-lang-cpp
            PLATFORMS linux)
