cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(camera_360_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common imgproc video)
add_executable(camera_360 camera_360.cc)

target_compile_definitions(camera_360 PUBLIC ${BoBRobotics_DEFINITIONS})
target_include_directories(camera_360 PUBLIC ${BoBRobotics_INCLUDE_DIRS})
target_link_libraries(camera_360 PUBLIC ${BoBRobotics_LIBRARIES})
