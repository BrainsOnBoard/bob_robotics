cmake_minimum_required(VERSION 3.10)
project(image_database_unwrapper)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)

add_executable(image_database_unwrapper image_database_unwrapper.cc)
target_link_libraries(image_database_unwrapper PUBLIC ${BoBRobotics_LIBRARIES})
