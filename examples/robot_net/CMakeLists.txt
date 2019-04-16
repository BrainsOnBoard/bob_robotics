cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(computer)
BoB_modules(common hid net os robots video)

target_link_libraries(computer PUBLIC $ENV{BOB_LINK_LIBS})