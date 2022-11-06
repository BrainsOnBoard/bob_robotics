
cmake_minimum_required(VERSION 3.20)
include(../../../../../cmake/BoBRobotics.cmake)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CUDA_ARCHITECTURES 86)
set(CMAKE_CUDA_COMPILER /usr/local/cuda-11.7/bin/nvcc)
project(ackermann_robot LANGUAGES CXX CUDA)
set(CMAKE_CUDA_STANDARD 14)

file(GLOB CPP_SOURCES *.cpp *.cu)
set(SOURCES ${CPP_SOURCES})


link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
set (PROJECT_LIBRARIES libCppDriver.so libgeneric_robot_window.so)
include_directories($ENV{WEBOTS_HOME}/include/controller/cpp)


find_package(BoBRobotics REQUIRED COMPONENTS imgproc common navigation video)


add_executable(dataset_evaluator_main dataset_evaluator_main.cu)
target_link_libraries(dataset_evaluator_main PUBLIC ${BoBRobotics_LIBRARIES} ${LIBRARIES} ${PROJECT_LIBRARIES} -lcublas)
set_target_properties(
        dataset_evaluator_main
        PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
