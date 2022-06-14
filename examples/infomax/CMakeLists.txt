cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(infomax_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common navigation)

foreach(TARGET infomax_matlab_comparison infomax_route_example)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach(TARGET infomax_matlab_comparison infomax_route_example)

find_package(BoBThirdParty REQUIRED COMPONENTS matplotlibcpp)
target_link_libraries(infomax_route_example PUBLIC BoBThirdParty::matplotlibcpp)

add_custom_command(TARGET infomax_matlab_comparison POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   "${CMAKE_CURRENT_SOURCE_DIR}/test_data" "${CMAKE_CURRENT_BINARY_DIR}/test_data")
