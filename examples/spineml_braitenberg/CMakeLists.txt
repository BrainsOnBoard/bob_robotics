cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(spineml_braitenberg_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid navigation robots/tank
             robots/control viz/sfml)
find_package(GeNN REQUIRED COMPONENTS spineml)

add_executable(spineml_braitenberg spineml_braitenberg.cc)
target_include_directories(spineml_braitenberg PUBLIC ${GeNN_INCLUDE_DIRS})
target_link_libraries(spineml_braitenberg PUBLIC ${BoBRobotics_LIBRARIES}
                      ${GeNN_LIBRARIES})
