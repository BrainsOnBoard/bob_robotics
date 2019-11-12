cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES shift.cc
            BOB_MODULES common
	    THIRD_PARTY third_party_modules)
