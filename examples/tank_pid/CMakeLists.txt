cmake_minimum_required(VERSION 3.10)
project(tank_pid_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid navigation net
             robots/tank robots/control vicon viz/sfml)
find_package(BoBThirdParty REQUIRED COMPONENTS matplotlibcpp)

add_executable(tank_pid tank_pid.cc)
target_link_libraries(tank_pid PUBLIC ${BoBRobotics_LIBRARIES}
                      ${BoBThirdParty_LIBRARIES})
