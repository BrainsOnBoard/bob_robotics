cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(plotting_example)

find_package(BoBThirdParty COMPONENTS matplotlibcpp REQUIRED)

add_executable(plotting plotting.cc)
target_compile_definitions(plotting PUBLIC ${BoBThirdParty_DEFINITIONS})
target_include_directories(plotting PUBLIC ${BoBThirdParty_INCLUDE_DIRS})
target_link_libraries(plotting PUBLIC ${BoBThirdParty_LIBRARIES})
