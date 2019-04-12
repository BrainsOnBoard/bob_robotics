cmake_minimum_required(VERSION 2.8)
project(test)

include(../../make_common/bob_robotics.cmake)
find_library(BOB_COMMON NAMES bob_common)

add_executable(test test.cc)
