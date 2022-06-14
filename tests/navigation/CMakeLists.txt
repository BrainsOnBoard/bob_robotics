cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(tests/navigation)

find_package(BoBRobotics QUIET COMPONENTS imgproc navigation video)
add_executable(generate_pm generate_pm.cc)
add_executable(generate_infomax generate_infomax.cc)
target_link_libraries(generate_pm PRIVATE ${BoBRobotics_LIBRARIES})
target_link_libraries(generate_infomax PRIVATE ${BoBRobotics_LIBRARIES})
