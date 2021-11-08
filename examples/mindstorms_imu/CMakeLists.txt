cmake_minimum_required(VERSION 3.10)
project(mindstorms_imu_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/ev3)

add_executable(mindstorms_imu mindstorms_imu.cc)
target_link_libraries(mindstorms_imu PUBLIC ${BoBRobotics_LIBRARIES})
