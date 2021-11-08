cmake_minimum_required(VERSION 3.10)
project(infomax_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS common navigation)

foreach(TARGET infomax_matlab_comparison infomax_route_example)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach(TARGET infomax_matlab_comparison infomax_route_example)

find_package(BoBThirdParty REQUIRED COMPONENTS matplotlibcpp)
target_link_libraries(infomax_route_example PUBLIC BoBThirdParty::matplotlibcpp)
