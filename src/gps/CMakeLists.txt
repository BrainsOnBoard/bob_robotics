cmake_minimum_required(VERSION 3.10)
project(gps)

find_package(BoBRobotics REQUIRED COMPONENTS common)
BoB_module(gps_reader.cc nmea_parser.cc)
