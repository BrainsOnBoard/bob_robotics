cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(WIN_EXE oculus
			SOURCES oculus.cc
			BOB_MODULES video
			EXTERNAL_LIBS opencv ovr_sdk
			PLATFORMS windows)
