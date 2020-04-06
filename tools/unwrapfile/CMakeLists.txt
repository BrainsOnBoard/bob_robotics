cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES unwrapfile.cc
            BOB_MODULES imgproc)

# Try to find ffmpeg
find_program(FFMPEG_PATH NAMES ffmpeg)
message(STATUS "ffmpeg path: ${FFMPEG_PATH}")
add_definitions(-DFFMPEG_PATH="${FFMPEG_PATH}")
