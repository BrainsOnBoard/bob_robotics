cmake_minimum_required(VERSION 2.8)
project(bob_common)

include(../../make_common/bob_robotics.cmake)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/lib)

include_directories(${BOB_ROBOTICS_PATH}/common
                    ${BOB_ROBOTICS_PATH}/third_party/plog/include)
file(GLOB COMMON_SRC
    "${BOB_ROBOTICS_PATH}/common/*.h"
    "*.cc"
)
add_library(bob_common STATIC ${COMMON_SRC})
target_compile_options(bob_common PRIVATE -std=c++14 -Wall -Wpedantic -Wextra -pthread)