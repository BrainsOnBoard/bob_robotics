cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(NAME robot_positioner
            BOB_MODULES common hid robots robots/control vicon)
