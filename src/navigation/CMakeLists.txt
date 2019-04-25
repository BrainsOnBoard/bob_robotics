cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_module(BOB_MODULES common imgproc
           EXTERNAL_LIBS opencv eigen3
           THIRD_PARTY matplotlibcpp)
