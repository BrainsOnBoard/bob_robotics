cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(infomax_route_example)
BoB_modules(common navigation)

target_link_libraries(infomax_route_example PUBLIC $ENV{BOB_LINK_LIBS})