#cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
#get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
#project(${PROJECT})

# Get C or C++ sources in the current directory (only).
#

# Link with the Webots controller library.


# Setup the target executable.


# Copy the target executable at the right location.
#add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
#  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
#)



cmake_minimum_required(VERSION 3.10)
include(../../../../cmake/BoBRobotics.cmake)
project(ackermann_robot)


file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${CPP_SOURCES})

link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
set (PROJECT_LIBRARIES libCppDriver.so libgeneric_robot_window.so)
include_directories($ENV{WEBOTS_HOME}/include/controller/cpp)


find_package(BoBRobotics QUIET REQUIRED COMPONENTS imgproc)


add_executable(ackermann_robot ackermann_robot.cpp)
target_link_libraries(ackermann_robot PUBLIC ${BoBRobotics_LIBRARIES} ${LIBRARIES} ${PROJECT_LIBRARIES})


#add_custom_command(TARGET ackermann_robot POST_BUILD COMMAND ${CMAKE_COMMAND} -E
#  copy ${CMAKE_BINARY_DIR}/ackermann_robot ${CMAKE_SOURCE_DIR}
#)

#cmake_minimum_required(VERSION 3.10)
#include(../../cmake/BoBRobotics.cmake)
#project(gazebo_tank_example)

#find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid robots/gazebo video)
#find_package(Gazebo REQUIRED)

#add_executable(gazebo_tank gazebo_tank.cc)
#target_link_libraries(gazebo_tank PUBLIC ${BoBRobotics_LIBRARIES})
#add_dependencies(gazebo_tank BoBRobotics::gazebo_tank_plugin)
