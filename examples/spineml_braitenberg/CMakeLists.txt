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

file(GLOB XML_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.xml")
foreach(FILE objects.yaml spineml_braitenberg.proj ${XML_FILES})
    configure_file(${FILE} "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
endforeach()
