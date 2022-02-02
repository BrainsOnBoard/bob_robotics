cmake_minimum_required(VERSION 3.10)
include(../../../../cmake/BoBRobotics.cmake)
project(robots/uav/bebop)

find_package(BoBRobotics REQUIRED COMPONENTS common video)
BoB_module(bebop.cc video.cc)

find_package(ARSDK REQUIRED)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${ARSDK_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${ARSDK_LIBRARIES})

# Disable some warnings from inside ARSDK
include(CheckCXXCompilerFlag)
foreach(FLAG -Wno-deprecated-declarations -Wno-implicit-fallthrough)
    check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORTS_${FLAG})
    if(COMPILER_SUPPORTS_${FLAG})
        target_compile_options(${BOB_MODULE_TARGET} PRIVATE ${FLAG})
    endif()
endforeach()
