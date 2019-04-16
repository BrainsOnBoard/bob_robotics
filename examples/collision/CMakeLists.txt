cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(NAME collision
            BOB_MODULES common hid net vicon robots robots/control viz/sfml_world)
