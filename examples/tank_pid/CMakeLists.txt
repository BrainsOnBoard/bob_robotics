cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(tank_pid)
BoB_modules(common hid net robots robots/control vicon viz/sfml_world)
BoB_third_party(matplotlibcpp)

target_link_libraries(tank_pid PUBLIC $ENV{BOB_LINK_LIBS})