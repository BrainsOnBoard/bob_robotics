cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(gazebo_uav_example)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/gazebo video)
find_package(Gazebo REQUIRED)

add_executable(gazebo_uav gazebo_uav.cc)
target_link_libraries(gazebo_uav PUBLIC ${BoBRobotics_LIBRARIES})
add_dependencies(gazebo_uav BoBRobotics::gazebo_uav_plugin)

set(BOB_GAZEBO_RESOURCES_PATH "${BOB_ROBOTICS_PATH}/resources/gazebo")

configure_file(iris_demo.world.in "${CMAKE_CURRENT_BINARY_DIR}/iris_demo.world")

# Only configure rothamsted.world.in if the reconstruction data is actually present
if(DEFINED ENV{ROTHAMSTED_3D_MODEL_PATH})
    set(ROTHAMSTED_3D_MODEL_PATH "$ENV{ROTHAMSTED_3D_MODEL_PATH}")
endif()
if(ROTHAMSTED_3D_MODEL_PATH)
    set(BOB_ROTHAMSTED_3D_MODEL_PATH "${ROTHAMSTED_3D_MODEL_PATH}")
    set(ROTHAMSTED_3D_MODEL_PATH "${ROTHAMSTED_3D_MODEL_PATH}" CACHE STRING "Path to 3D Rothamsted reconstruction")

    configure_file(rothamsted.world.in "${CMAKE_CURRENT_BINARY_DIR}/rothamsted.world")
else()
    message(WARNING "ROTHAMSTED_3D_MODEL_PATH is not set. Rothamsted simulation will not work.")
endif()

if(UNIX)
    configure_file(run.sh.in "${CMAKE_CURRENT_BINARY_DIR}/run.sh")
    execute_process(COMMAND chmod +x "${CMAKE_CURRENT_BINARY_DIR}/run.sh")
endif()
