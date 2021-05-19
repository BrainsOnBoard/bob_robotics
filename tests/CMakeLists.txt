cmake_minimum_required(VERSION 3.1)
include(../cmake/bob_robotics.cmake)
BoB_project(EXECUTABLE tests
            SOURCES circstat.cc differencers.cc geometry.cc image_database.cc
                    infomax.cc opencv_unwrap_360_serialisation.cc
                    perfect_memory.cc string.cc tests.cc
            BOB_MODULES imgproc navigation video
            EXTERNAL_LIBS gtest eigen3)
