cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(image_database_unwrapper)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)

add_executable(image_database_unwrapper image_database_unwrapper.cc)
target_link_libraries(image_database_unwrapper PUBLIC ${BoBRobotics_LIBRARIES})
