cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(lm9ds1)
BoB_modules(common)

target_link_libraries(lm9ds1 PUBLIC $ENV{BOB_LINK_LIBS})