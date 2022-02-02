cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/tank)

find_package(BoBRobotics REQUIRED)
BoB_module(norbot.cc)
