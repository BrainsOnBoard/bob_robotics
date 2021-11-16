cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(navigation)

find_package(BoBRobotics REQUIRED COMPONENTS common imgproc)
BoB_module(image_database.cc perfect_memory_window.cc read_objects.cc)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_INCLUDE_DIRS}
                           ${TBB_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC Eigen3::Eigen ${OpenCV_LIBS}
                      ${TBB_LIBRARIES})

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${BOB_MODULE_TARGET} PUBLIC OpenMP::OpenMP_CXX)
endif()
