cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(spineml_braitenberg_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid navigation robots/tank
             robots/control viz/sfml)

# TODO: Link against spineml stuff
add_executable(spineml_braitenberg spineml_braitenberg.cc)
target_link_libraries(spineml_braitenberg PUBLIC ${BoBRobotics_LIBRARIES})


# BoB_project(SOURCES spineml_braitenberg.cc
#             BOB_MODULES common
#             EXTERNAL_LIBS spineml_simulation)
