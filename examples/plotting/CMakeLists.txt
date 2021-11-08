cmake_minimum_required(VERSION 3.10)
project(plotting_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBThirdParty COMPONENTS matplotlibcpp REQUIRED)

add_executable(plotting plotting.cc)
target_compile_definitions(plotting PUBLIC ${BoBThirdParty_DEFINITIONS})
target_include_directories(plotting PUBLIC ${BoBThirdParty_INCLUDE_DIRS})
target_link_libraries(plotting PUBLIC ${BoBThirdParty_LIBRARIES})
