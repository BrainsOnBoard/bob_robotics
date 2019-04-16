cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_module(os)
BoB_modules(common)

find_package(OpenCV REQUIRED)
target_link_libraries(bob_os PUBLIC ${OpenCV_LIBS})
