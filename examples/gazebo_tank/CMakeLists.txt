cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gazebo_tank_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/gazebo video)
find_package(Gazebo REQUIRED)

add_executable(gazebo_tank gazebo_tank.cc)
target_link_libraries(gazebo_tank PUBLIC ${BoBRobotics_LIBRARIES})
add_dependencies(gazebo_tank BoBRobotics::gazebo_tank_plugin)

set(BOB_GAZEBO_RESOURCES_PATH "${BOB_ROBOTICS_PATH}/resources/gazebo")

set(BOB_GAZEBO_CAMERA simple_camera)
configure_file(differential_drive.world.in "${CMAKE_CURRENT_BINARY_DIR}/differential_drive.world")

set(BOB_GAZEBO_CAMERA panoramic_camera)
configure_file(differential_drive.world.in "${CMAKE_CURRENT_BINARY_DIR}/differential_drive_panoramic.world")

if(UNIX)
    configure_file(run.sh.in "${CMAKE_CURRENT_BINARY_DIR}/run.sh")
    execute_process(COMMAND chmod +x "${CMAKE_CURRENT_BINARY_DIR}/run.sh")
endif()
