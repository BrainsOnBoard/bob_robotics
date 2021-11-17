cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(display)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common video)

foreach(TARGET display unwrap_display)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
