cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(NAME norbot_control
            BOB_MODULES common hid net os robots video
            PLATFORMS linux)
