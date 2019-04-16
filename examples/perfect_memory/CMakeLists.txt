cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(perfect_memory_test)
BoB_modules(navigation)

target_link_libraries(perfect_memory_test PUBLIC $ENV{BOB_LINK_LIBS})