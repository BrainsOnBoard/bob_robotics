cmake_minimum_required(VERSION 3.1)

if(NOT PROJECT_NAME)
    message(FATAL_ERROR "project() should be called before including this file")
endif()

# Add each subdirectory with a CMakeLists.txt file
file(GLOB children "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(child IN LISTS children)
    if(IS_DIRECTORY "${child}" AND EXISTS "${child}/CMakeLists.txt")
        get_filename_component(dirname ${child} NAME)
        add_subdirectory(${child} ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${dirname})
    endif()
endforeach()

if(NOT BOB_DIR)
    # Build modules + third-party libs in here
    set(BOB_DIR "${CMAKE_CURRENT_BINARY_DIR}/BoB")

    # Output binaries to current folder
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}")
endif()
