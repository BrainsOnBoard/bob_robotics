cmake_minimum_required(VERSION 3.10)
project(dataset_recorder)

set(IS_EXPERIMENT TRUE)
include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS gps imgproc navigation
             robots/ackermann video)

add_executable(dataset_recorder dataset_recorder.cc)
target_link_libraries(dataset_recorder PUBLIC ${BoBRobotics_LIBRARIES})
if(DUMMY_CAMERA)
    target_compile_definitions(dataset_recorder PUBLIC DUMMY_CAMERA)
endif()
