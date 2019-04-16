cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(pure_pursuit_controller)
BoB_modules(robots/control viz/car_display)

target_link_libraries(pure_pursuit_controller PUBLIC $ENV{BOB_LINK_LIBS})