cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gantry)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/gantry video)
foreach(EXE gantry gantry_log_velocities)
    add_executable(${EXE} ${EXE}.cc)
    target_link_libraries(${EXE} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
