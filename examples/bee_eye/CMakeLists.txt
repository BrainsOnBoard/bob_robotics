cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(bee_eye_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc video)

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
                  COMMAND ${BoBRobotics_ROOT_DIR}/bin/make_bee_eye_data_h.py
                          ${INPUT_WIDTH} ${INPUT_HEIGHT})

add_executable(bee_eye bee_eye.cc)
add_dependencies(bee_eye bee_eye_data.h)
target_link_libraries(bee_eye PUBLIC ${BoBRobotics_LIBRARIES})

# So that the program can find the header
target_include_directories(bee_eye PUBLIC "${CMAKE_BINARY_DIR}")
