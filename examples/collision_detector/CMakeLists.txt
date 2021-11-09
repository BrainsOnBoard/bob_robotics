cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(collision_detector_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid navigation net
             robots/tank/net robots/control vicon viz/sfml)

add_executable(collision_detector collision_detector.cc)
target_link_libraries(collision_detector PUBLIC ${BoBRobotics_LIBRARIES})
