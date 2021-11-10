cmake_minimum_required(VERSION 3.10)
include(../../../../cmake/BoBRobotics.cmake)
project(robots/uav/bebop)

find_package(BoBRobotics REQUIRED COMPONENTS common video)
BoB_module(bebop.cc video.cc)

find_package(ARSDK REQUIRED)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${ARSDK_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${ARSDK_LIBRARIES})

# We use a bunch of deprecated ffmpeg functions in this module
target_compile_options(${BOB_MODULE_TARGET} PRIVATE -Wno-deprecated-declarations)
