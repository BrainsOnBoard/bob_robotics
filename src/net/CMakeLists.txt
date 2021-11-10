cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(net)

find_package(BoBRobotics REQUIRED COMPONENTS common os)
BoB_module(client.cc connection.cc imu_netsource.cc server.cc socket.cc)
