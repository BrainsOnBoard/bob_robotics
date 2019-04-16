cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(bebop)
BoB_modules(robots/bebop common hid video)

target_link_libraries(bebop PUBLIC $ENV{BOB_LINK_LIBS})