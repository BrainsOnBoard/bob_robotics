cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES background_exception_catcher.cc i2c_interface.cc
                   lm9ds1_imu.cc logging.cc pid.cc semaphore.cc stopwatch.cc
                   threadable.cc
           EXTERNAL_LIBS i2c)
