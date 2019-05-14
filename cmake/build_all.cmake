cmake_minimum_required(VERSION 3.1)
project(build_all)

get_filename_component(root_dirname ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Building BoB robotics ${root_dirname}...")

# Build modules + third-party libs in here
set(BOB_DIR "${CMAKE_CURRENT_BINARY_DIR}/BoB")

# Add each subdirectory with a CMakeLists.txt file
file(GLOB children "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(child IN LISTS children)
    if(IS_DIRECTORY "${child}" AND EXISTS "${child}/CMakeLists.txt")
        get_filename_component(dirname ${child} NAME)
        message("Adding project ${dirname}")
        add_subdirectory(${child} ${CMAKE_CURRENT_BINARY_DIR}/${root_dirname}/${dirname})
    endif()
endforeach()

# Output binaries to current folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}")
