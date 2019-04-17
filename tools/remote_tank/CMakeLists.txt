cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(NAME remote_tank
            BOB_MODULES common hid net robots
            PLATFORMS linux windows)
