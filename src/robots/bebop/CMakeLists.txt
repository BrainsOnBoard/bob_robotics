cmake_minimum_required(VERSION 3.1)
include(../../../make_common/common.cmake)
BoB_module(robots/bebop)

# Needed, otherwise we get warnings from inside ARSDK
add_compile_flags("-Wno-deprecated-declarations -Wno-implicit-fallthrough")
