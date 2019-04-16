cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(camera_360)
BoB_modules(common imgproc video)

target_link_libraries(camera_360 PUBLIC $ENV{BOB_LINK_LIBS})