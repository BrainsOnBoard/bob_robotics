cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(display_test)
BoB_modules(common video)

target_link_libraries(display_test PUBLIC $ENV{BOB_LINK_LIBS})