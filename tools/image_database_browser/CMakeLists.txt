cmake_minimum_required(VERSION 3.10)
project(image_database_browser)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)

add_executable(image_database_browser image_database_browser.cc)
target_link_libraries(image_database_browser PUBLIC ${BoBRobotics_LIBRARIES})
