cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(image_database_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation video)

foreach(TARGET read_example write_example)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
