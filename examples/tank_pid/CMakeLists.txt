cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(BOB_MODULES common hid net robots robots/control vicon viz/sfml_world
            THIRD_PARTY matplotlibcpp)
