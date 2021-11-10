cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/omni2d)

find_package(BoBRobotics REQUIRED)
BoB_module(mecanum.cc mecanum_pca_9685.cc)
