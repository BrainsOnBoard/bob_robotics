cmake_minimum_required(VERSION 3.10)
project(vicon)

find_package(BoBRobotics REQUIRED COMPONENTS common net)
BoB_module(capture_control.cc udp.cc)
