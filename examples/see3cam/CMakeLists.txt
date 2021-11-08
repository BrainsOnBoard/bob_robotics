cmake_minimum_required(VERSION 3.10)
project(see3cam_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc video)

add_executable(see3cam see3cam.cc)
target_link_libraries(see3cam PUBLIC ${BoBRobotics_LIBRARIES})
