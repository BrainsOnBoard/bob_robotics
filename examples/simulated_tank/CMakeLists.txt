cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(simulated_tank)
BoB_modules(os hid robots viz/sfml_world)

target_link_libraries(simulated_tank PUBLIC $ENV{BOB_LINK_LIBS})