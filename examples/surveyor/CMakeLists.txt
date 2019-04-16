cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(surveyor)
BoB_modules(hid robots)

target_link_libraries(surveyor PUBLIC $ENV{BOB_LINK_LIBS})