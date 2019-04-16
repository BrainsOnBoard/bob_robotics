cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(robot_positioner)
BoB_modules(common hid robots robots/control vicon)

target_link_libraries(robot_positioner PUBLIC $ENV{BOB_LINK_LIBS})