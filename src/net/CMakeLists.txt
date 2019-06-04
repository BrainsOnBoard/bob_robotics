cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES client.cc connection.cc imu_netsource.cc server.cc socket.cc
           BOB_MODULES common os)
