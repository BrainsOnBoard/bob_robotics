cmake_minimum_required(VERSION 3.10)
project(robots/ackermann)

find_package(BoBRobotics REQUIRED COMPONENTS common)
BoB_module(rc_car_bot.cc simulated_ackermann.cc)
