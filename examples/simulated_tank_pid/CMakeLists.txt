cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(simulated_tank_pid)
BoB_modules(hid robots robots/control viz/sfml_world)

target_link_libraries(simulated_tank_pid PUBLIC $ENV{BOB_LINK_LIBS})