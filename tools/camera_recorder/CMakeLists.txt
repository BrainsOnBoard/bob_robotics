cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(camera_recorder)

set(MODULES hid imgproc robots/tank video)
if(USE_VICON)
    list(APPEND MODULES vicon)
    add_definitions(-DUSE_VICON)
endif()

find_package(BoBRobotics QUIET REQUIRED COMPONENTS ${MODULES})

add_executable(camera_recorder camera_recorder.cc)
target_link_libraries(camera_recorder PUBLIC ${BoBRobotics_LIBRARIES})
