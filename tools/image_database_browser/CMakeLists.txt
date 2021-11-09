cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(image_database_browser)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)

add_executable(image_database_browser image_database_browser.cc)
target_link_libraries(image_database_browser PUBLIC ${BoBRobotics_LIBRARIES})
