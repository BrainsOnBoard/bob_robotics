cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(vicon_test)
BoB_modules(vicon)

target_link_libraries(vicon_test PUBLIC $ENV{BOB_LINK_LIBS})