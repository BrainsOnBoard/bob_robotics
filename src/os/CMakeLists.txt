cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(os)

find_package(BoBRobotics REQUIRED COMPONENTS common)

# TODO: screen.cc needs X11
BoB_module(net.cc video.cc)
