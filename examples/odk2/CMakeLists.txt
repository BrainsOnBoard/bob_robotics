cmake_minimum_required(VERSION 3.10)
project(odk2_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS video/odk2)

add_executable(odk2 odk2.cc)
target_link_libraries(odk2 PUBLIC ${BoBRobotics_LIBRARIES})
