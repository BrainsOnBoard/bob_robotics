cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(hid)

find_package(BoBRobotics REQUIRED COMPONENTS common)
if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
    BoB_module(${BOB_MODULE_TARGET} STATIC joystick_linux.cc)
elseif(WIN32)
    BoB_module(${BOB_MODULE_TARGET} STATIC joystick_windows.cc)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()
