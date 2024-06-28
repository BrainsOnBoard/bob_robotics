cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(hid/net)

find_package(BoBRobotics REQUIRED net)
BoB_module(sink.cc source.cc)
