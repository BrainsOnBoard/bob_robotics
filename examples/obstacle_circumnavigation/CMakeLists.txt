cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(obstacle_circumnavigation_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid navigation net
             robots/tank robots/tank/net robots/control vicon viz/sfml)

configure_file(objects.yaml "${CMAKE_CURRENT_BINARY_DIR}/objects.yaml" COPYONLY)

set(DEPS BoBRobotics::common BoBRobotics::navigation
    BoBRobotics::robots__control BoBRobotics::viz__sfml)
foreach(TYPE avoiding_positioner circumnavigation)
    set(TARGET_SIM simulated_obstacle_${TYPE})
    add_executable(${TARGET_SIM} ${TARGET_SIM}.cc)
    target_link_libraries(${TARGET_SIM} PUBLIC ${DEPS} BoBRobotics::robots__tank)

    set(TARGET_ROBOT vicon_obstacle_${TYPE})
    add_executable(${TARGET_ROBOT} ${TARGET_ROBOT}.cc)
    target_link_libraries(${TARGET_ROBOT} PUBLIC ${DEPS} BoBRobotics::net
                          BoBRobotics::robots__tank__net BoBRobotics::vicon)
endforeach(TYPE avoiding_positioner circumnavigation)

configure_file(objects.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
