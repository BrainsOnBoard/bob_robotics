cmake_minimum_required(VERSION 3.10)
project(pure_pursuit_controller_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS robots/ackermann
             robots/control viz/car_display)
add_executable(pure_pursuit_controller pure_pursuit_controller.cc)
target_link_libraries(pure_pursuit_controller PUBLIC ${BoBRobotics_LIBRARIES})
