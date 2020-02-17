cmake_minimum_required(VERSION 3.1)
include(../../../cmake/bob_robotics.cmake)
BoB_project(SOURCES WaveletSandbox.cpp
			EXTERNAL_LIBS opencv
			THIRD_PARTY wavelet2s)
