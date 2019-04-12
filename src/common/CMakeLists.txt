cmake_minimum_required(VERSION 2.8)
include(../../make_common/module.cmake)
BoB_build_module(common)
find_library(I2C NAMES i2c)
