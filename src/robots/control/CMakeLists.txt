cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/control)

find_package(BoBRobotics REQUIRED COMPONENTS common)
BoB_module(collision_detector.cc pure_pursuit_controller.cc)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${EIGEN3_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_LIBS})
