cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(unwrapfile)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc)

# Try to find ffmpeg
find_program(FFMPEG_PATH NAMES ffmpeg)
if(FFMPEG_PATH)
    message(STATUS "ffmpeg path: ${FFMPEG_PATH}")
    add_definitions(-DFFMPEG_PATH="${FFMPEG_PATH}")
endif()

add_executable(unwrapfile unwrapfile.cc)
target_link_libraries(unwrapfile PUBLIC ${BoBRobotics_LIBRARIES})
