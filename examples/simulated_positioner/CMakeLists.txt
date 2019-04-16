cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(simulated_positioner)
BoB_modules(robots robots/control viz/sfml_world)

target_link_libraries(simulated_positioner PUBLIC $ENV{BOB_LINK_LIBS})