cmake_minimum_required(VERSION 3.10)
project(remote_imu_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/tank/net)

add_executable(remote_imu remote_imu.cc)
target_link_libraries(remote_imu PUBLIC ${BoBRobotics_LIBRARIES})
