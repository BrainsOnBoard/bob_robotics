cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(pure_pursuit_controller_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/ackermann
             robots/control viz/sfml)
add_executable(pure_pursuit_controller pure_pursuit_controller.cc)
target_link_libraries(pure_pursuit_controller PUBLIC ${BoBRobotics_LIBRARIES})
