cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(bebop_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/uav/bebop video)

foreach(TARGET bebop bebop_relative_move)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
