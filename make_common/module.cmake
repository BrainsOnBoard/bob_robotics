cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/lib)

function(BoB_build_module NAME)
    set(MODULE_NAME bob_${NAME})
    project(${MODULE_NAME})

    include_directories(${BOB_ROBOTICS_PATH}/include)

    file(GLOB SRC_FILES
        "${BOB_ROBOTICS_PATH}/include/${NAME}/*.h"
        "*.cc"
    )
    add_library(${MODULE_NAME} STATIC ${SRC_FILES})
endfunction()
