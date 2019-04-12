cmake_minimum_required(VERSION 2.8)

function(ADD_CXXFLAGS EXTRA_ARGS)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_ARGS}")
endfunction()

function(ADD_LDFLAGS EXTRA_ARGS)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_ARGS}")
endfunction()

set(BOB_ROBOTICS_PATH "${CMAKE_CURRENT_LIST_DIR}/..")
include_directories(${BOB_ROBOTICS_PATH}/include
                    ${BOB_ROBOTICS_PATH}/third_party/plog/include)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    ADD_CXXFLAGS("-std=c++14 -Wall -Wpedantic -Wextra")
    ADD_LDFLAGS("-pthread")
endif()

link_directories(${BOB_ROBOTICS_PATH}/lib)
