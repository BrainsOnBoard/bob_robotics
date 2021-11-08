cmake_minimum_required(VERSION 3.10)
project(bebop_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/uav/bebop video)

foreach(TARGET bebop bebop_relative_move)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
