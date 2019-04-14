cmake_minimum_required(VERSION 3.1)
include(../../make_common/module.cmake)
ADD_CXXFLAGS("-Wno-deprecated-declarations -Wno-implicit-fallthrough")
BoB_build_module(bebop)

include_directories($ENV{ARSDK_ROOT}/out/arsdk-native/staging/include)

find_package(OpenCV REQUIRED)
target_link_libraries(bob_bebop PUBLIC ${OpenCV_LIBS})

# install(TARGETS bob_bebop DESTINATION ${BOB_ROBOTICS_PATH}/lib)