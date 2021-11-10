cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(video)

find_package(BoBRobotics REQUIRED COMPONENTS common os net imgproc)
BoB_module(display.cc input.cc netsink.cc netsource.cc opencvinput.cc
           panoramic.cc rpi_cam.cc see3cam_cu40.cc v4l_camera.cc)

find_package(OpenCV REQUIRED)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_LIBS})
