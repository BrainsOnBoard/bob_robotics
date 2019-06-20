cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES background_exception_catcher.cc geometry.cc get_new_path.cc
                   i2c_interface.cc lm9ds1_imu.cc logging.cc macros.cc pid.cc
                   semaphore.cc serial_interface.cc stopwatch.cc threadable.cc
           EXTERNAL_LIBS eigen3 i2c)

# Some versions of gcc complain about unused return values in
# serial_interface.cc and it seems basically impossible to indicate that you
# really do want to throw the value away, so disable this warning.
if(GNU_TYPE_COMPILER)
    add_definitions(-Wno-unused-result)
endif()
