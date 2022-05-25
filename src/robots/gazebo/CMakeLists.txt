cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/gazebo)

find_package(BoBRobotics REQUIRED)
BoB_module(node.cc tank.cc uav.cc rc_car.cc)

find_package(gazebo REQUIRED)
list(REMOVE_ITEM GAZEBO_LIBRARIES debug)
list(REMOVE_ITEM GAZEBO_LIBRARIES optimized)
list(REMOVE_DUPLICATES GAZEBO_LIBRARIES)

target_include_directories(${BOB_MODULE_TARGET} PUBLIC "${GAZEBO_INCLUDE_DIRS}")
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC "${GAZEBO_LIBRARIES}")

foreach(PLUGIN tank_plugin uav_plugin rc_car_plugin)
    add_library(gazebo_${PLUGIN} SHARED ${PLUGIN}.cc)
    set_target_properties(gazebo_${PLUGIN} PROPERTIES
                          EXCLUDE_FROM_ALL TRUE
                          LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_library(BoBRobotics::gazebo_${PLUGIN} ALIAS gazebo_${PLUGIN})

    target_include_directories(gazebo_${PLUGIN} PUBLIC "${GAZEBO_INCLUDE_DIRS}")
    target_link_libraries(gazebo_${PLUGIN} PUBLIC "${GAZEBO_LIBRARIES}" BoBRobotics::base)
endforeach(PLUGIN tank_plugin uav_plugin rc_car_plugin)

# Newer versions of Gazebo require C++17
if("${GAZEBO_MAJOR_VERSION}" GREATER_EQUAL 11)
    set_target_properties(bob_robots__gazebo gazebo_tank_plugin gazebo_uav_plugin gazebo_rc_car_plugin
                          PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED TRUE)
endif()

# Disable warning from inside gazebo
include(CheckCXXCompilerFlag)
foreach(FLAG -Wno-deprecated-copy)
    check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORTS_${FLAG})
    if(COMPILER_SUPPORTS_${FLAG})
        target_compile_options(gazebo_uav_plugin PRIVATE ${FLAG})
    endif()
endforeach()
