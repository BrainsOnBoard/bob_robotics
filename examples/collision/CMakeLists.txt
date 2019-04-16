cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(collision)
BoB_modules(common hid net vicon robots robots/control viz/sfml_world)

target_link_libraries(collision PUBLIC $ENV{BOB_LINK_LIBS})