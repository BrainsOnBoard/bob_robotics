cmake_minimum_required(VERSION 3.1)
include(../cmake/bob_robotics.cmake)
BoB_project(SOURCES tests.cc
            BOB_MODULES imgproc
            EXTERNAL_LIBS gtest eigen3)

# We need to run a script to generate a header file before compiling
file(GLOB headers ${CMAKE_SOURCE_DIR}/test_headers/*.h)
add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/.include_test_headers.h
                   DEPENDS ${headers}
                   COMMAND ${CMAKE_SOURCE_DIR}/make_tests_include_header.sh
                   COMMENT "Generating header file for tests")
add_custom_target(test_headers ALL DEPENDS ${CMAKE_SOURCE_DIR}/.include_test_headers.h)
add_dependencies(tests test_headers)
