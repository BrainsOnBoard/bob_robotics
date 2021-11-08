cmake_minimum_required(VERSION 3.10)
project(ant_world_db_creator)

set(IS_EXPERIMENT TRUE)
include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS antworld navigation video)

foreach(TARGET ant_world_db_creator ant_world_map_creator)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
