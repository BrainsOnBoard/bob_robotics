cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(ant_agent)
BoB_modules(common hid antworld video)

target_link_libraries(ant_agent PUBLIC $ENV{BOB_LINK_LIBS})