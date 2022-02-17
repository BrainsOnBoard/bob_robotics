cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gantry_dvs)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common robots/gantry)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cc)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${BoBRobotics_LIBRARIES} ${OpenCV_LIBS})
