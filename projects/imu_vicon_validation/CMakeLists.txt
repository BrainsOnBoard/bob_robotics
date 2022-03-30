cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(imu_vicon_validation)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid robots/omni2d vicon)
add_executable(imu_vicon_validation imu_vicon_validation.cc)
target_link_libraries(imu_vicon_validation PRIVATE ${BoBRobotics_LIBRARIES})
