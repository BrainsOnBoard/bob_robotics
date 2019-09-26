cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES background_exception_catcher.cc geometry.cc i2c_interface.cc
                   lm9ds1_imu.cc logging.cc macros.cc path.cc pid.cc
                   semaphore.cc serial_interface.cc stopwatch.cc threadable.cc
           EXTERNAL_LIBS eigen3 i2c)
