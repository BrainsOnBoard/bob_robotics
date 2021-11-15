cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/tank)

find_package(BoBRobotics REQUIRED)
BoB_module(norbot.cc)

if(NOT DEFINED DUMMY_TANK_VERBOSITY)
    set(DUMMY_TANK_VERBOSITY debug)
endif()

string(TOLOWER ${DUMMY_TANK_VERBOSITY} DUMMY_TANK_VERBOSITY)
set(DUMMY_TANK_VERBOSITY ${DUMMY_TANK_VERBOSITY} CACHE STRING "Logging verbosity level for DummyTank motor commands" FORCE)
target_compile_definitions(${BOB_MODULE_TARGET} PUBLIC -DDUMMY_TANK_VERBOSITY=${DUMMY_TANK_VERBOSITY})
