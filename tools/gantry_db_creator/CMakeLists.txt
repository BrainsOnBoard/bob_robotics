cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gantry_db_creator)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/gantry video)
add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC ${BoBRobotics_LIBRARIES})
