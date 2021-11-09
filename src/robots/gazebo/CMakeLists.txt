cmake_minimum_required(VERSION 3.10)
project(robots/gazebo)

find_package(BoBRobotics REQUIRED)
BoB_module(node.cc tank.cc uav.cc)

find_package(gazebo REQUIRED)
list(REMOVE_ITEM GAZEBO_LIBRARIES debug)
list(REMOVE_ITEM GAZEBO_LIBRARIES optimized)
list(REMOVE_DUPLICATES GAZEBO_LIBRARIES)

target_include_directories(${BOB_MODULE_TARGET} PUBLIC "${GAZEBO_INCLUDE_DIRS}")
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC "${GAZEBO_LIBRARIES}")

foreach(PLUGIN tank_plugin uav_plugin)
    add_library(gazebo_${PLUGIN} SHARED ${PLUGIN}.cc)
    set_target_properties(gazebo_${PLUGIN} PROPERTIES
                          EXCLUDE_FROM_ALL TRUE
                          LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_library(BoBRobotics::gazebo_${PLUGIN} ALIAS gazebo_${PLUGIN})

    target_include_directories(gazebo_${PLUGIN} PUBLIC "${GAZEBO_INCLUDE_DIRS}")
    target_link_libraries(gazebo_${PLUGIN} PUBLIC "${GAZEBO_LIBRARIES}" BoBRobotics::base)
endforeach(PLUGIN tank_plugin uav_plugin)
