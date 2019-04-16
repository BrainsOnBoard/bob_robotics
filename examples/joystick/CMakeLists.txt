cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(joystick_test)
BoB_modules(hid)

target_link_libraries(joystick_test PUBLIC $ENV{BOB_LINK_LIBS})