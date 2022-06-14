cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(tank_pid_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid navigation net
             robots/tank robots/control vicon viz/sfml)
find_package(BoBThirdParty REQUIRED COMPONENTS matplotlibcpp)

add_executable(tank_pid tank_pid.cc)
target_link_libraries(tank_pid PUBLIC ${BoBRobotics_LIBRARIES}
                      ${BoBThirdParty_LIBRARIES})
