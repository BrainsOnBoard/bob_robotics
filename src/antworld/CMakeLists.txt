cmake_minimum_required(VERSION 3.1)
include(../../make_common/module.cmake)
BoB_build_module(antworld)

find_package(OpenCV REQUIRED)
target_link_libraries(bob_antworld PUBLIC ${OpenCV_LIBS})
