cmake_minimum_required(VERSION 3.10)

set(IS_EXPERIMENT TRUE)
include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid navigation robots/uav/bebop)
find_package(BoBThirdParty REQUIRED matplotlibcpp)

add_executable(uav_navigation uav_navigation.cc)
target_link_libraries(uav_navigation PUBLIC ${BoBRobotics_LIBRARIES}
                      ${BoBThirdParty_LIBRARIES})
