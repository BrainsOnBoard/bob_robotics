cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(vicon_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS vicon)
find_package(BoBThirdParty COMPONENTS matplotlibcpp)

foreach(TARGET vicon vicon_velocity)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC BoBRobotics::vicon)
endforeach()

if(BoBThirdParty_FOUND)
    add_executable(vicon_plot vicon_plot.cc)
    target_link_libraries(vicon_plot PUBLIC BoBRobotics::vicon
                          BoBThirdParty::matplotlibcpp)
else()
    message(WARNING "Could not find matplotlibcpp's dependencies; not building vicon_plot")
endif()
