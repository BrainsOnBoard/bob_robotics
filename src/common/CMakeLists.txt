cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES background_exception_catcher.cc bn055_imu.cc geometry.cc
                   i2c_interface.cc lm9ds1_imu.cc macros.cc main.cc path.cc
                   pca_9685.cc pid.cc semaphore.cc serial_interface.cc stopwatch.cc
                   string.cc threadable.cc
           EXTERNAL_LIBS eigen3 i2c)
