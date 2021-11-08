cmake_minimum_required(VERSION 3.10)
project(viz)

find_package(BoBRobotics REQUIRED)
find_package(BoBThirdParty REQUIRED COMPONENTS matplotlibcpp)

BoB_module(plot_ridf.cc)
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${BoBThirdParty_LIBRARIES})
