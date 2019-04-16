cmake_minimum_required(VERSION 3.1)
include(../../../make_common/common.cmake)
ADD_CXXFLAGS("-Wno-deprecated-declarations -Wno-implicit-fallthrough")
BoB_module(robots/bebop)
