cmake_minimum_required(VERSION 2.8)
include(../../make_common/module.cmake)
ADD_CXXFLAGS("-Wno-deprecated-declarations -Wno-implicit-fallthrough")
BoB_build_module(bebop)

include_directories($ENV{ARSDK_ROOT}/out/arsdk-native/staging/include)
#link_directories($ENV{ARSDK_ROOT}/out/arsdk-native/staging/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv)
target_link_libraries(bob_bebop ${OPENCV_LIBRARIES})
target_include_directories(bob_bebop PUBLIC ${OPENCV_INCLUDE_DIRS})
target_compile_options(bob_bebop PUBLIC ${OPENCV_CFLAGS_OTHER})
