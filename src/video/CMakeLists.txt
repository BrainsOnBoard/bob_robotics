cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES display.cc input.cc netsink.cc netsource.cc opencvinput.cc
                   panoramic.cc rpi_cam.cc see3cam_cu40.cc v4l_camera.cc
           BOB_MODULES common os net imgproc
           EXTERNAL_LIBS opencv)
