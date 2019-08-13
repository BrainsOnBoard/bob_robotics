cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES bee_eye.cc
            BOB_MODULES imgproc video)

# Default to 800x600 for input resolution
if(NOT INPUT_WIDTH)
    set(INPUT_WIDTH 800)
endif()
if(NOT INPUT_HEIGHT)
    set(INPUT_HEIGHT 600)
endif()
add_definitions(-DINPUT_WIDTH=${INPUT_WIDTH} -DINPUT_HEIGHT=${INPUT_HEIGHT})
message(STATUS "Input resolution for bee eye: [${INPUT_WIDTH} x ${INPUT_HEIGHT}]")

# Auto-generate header file
add_custom_target(bee_eye_data.h
                  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
                  COMMAND ${BOB_ROBOTICS_PATH}/bin/make_bee_eye_data_h.py
                          ${INPUT_WIDTH} ${INPUT_HEIGHT})
foreach(target IN LISTS BOB_TARGETS)
    add_dependencies(${target} bee_eye_data.h)
endforeach()

# So that the program can find the header
BoB_add_include_directories("${CMAKE_BINARY_DIR}")
