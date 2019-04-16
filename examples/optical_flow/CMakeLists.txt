cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(optical_flow)
BoB_modules(common imgproc video)

target_link_libraries(optical_flow PUBLIC $ENV{BOB_LINK_LIBS})