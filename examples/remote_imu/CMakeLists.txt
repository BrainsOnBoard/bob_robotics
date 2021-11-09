cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(remote_imu_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/tank/net)

add_executable(remote_imu remote_imu.cc)
target_link_libraries(remote_imu PUBLIC ${BoBRobotics_LIBRARIES})
