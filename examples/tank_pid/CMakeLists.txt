cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES tank_pid.cc
            BOB_MODULES common hid navigation net robots robots/control vicon
                        viz/sfml_world
            THIRD_PARTY matplotlibcpp)

if(GNU_TYPE_COMPILER)
    # gcc is really stubborn about us not wanting to use the return value of a
    # call to system()
    add_compile_flags("-Wno-unused-result")
endif()
