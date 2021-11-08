cmake_minimum_required(VERSION 3.10)
project(simulated_tank_pid_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid robots/tank
             robots/control viz/sfml)

add_executable(simulated_tank_pid simulated_tank_pid.cc)
target_link_libraries(simulated_tank_pid PUBLIC ${BoBRobotics_LIBRARIES})
