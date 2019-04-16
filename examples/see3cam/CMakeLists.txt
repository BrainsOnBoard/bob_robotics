cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(see3cam)
BoB_modules(common imgproc video)

target_link_libraries(see3cam PUBLIC $ENV{BOB_LINK_LIBS})