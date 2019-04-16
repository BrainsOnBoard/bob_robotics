cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(plotting)
BoB_third_party(matplotlibcpp)

target_link_libraries(plotting PUBLIC $ENV{BOB_LINK_LIBS})