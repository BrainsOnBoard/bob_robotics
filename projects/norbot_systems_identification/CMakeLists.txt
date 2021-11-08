cmake_minimum_required(VERSION 3.10)
project(norbot_systems_identification)

set(IS_EXPERIMENT TRUE)
include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid net robots/tank
             robots/tank/net vicon)

foreach(TARGET angular_velocity robot speed_test)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach()
