cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(image_database_to_video)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)

add_executable(image_database_to_video image_database_to_video.cc)
target_link_libraries(image_database_to_video PUBLIC ${BoBRobotics_LIBRARIES})
