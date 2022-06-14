cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(rc_car_bot_example)

find_package(BoBRobotics QUIET REQUIRED robots/ackermann)

foreach(TARGET rc_car_bot rc_car_bot_i2c_control rc_car_bot_passive)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach(TARGET rc_car_bot rc_car_bot_i2c_control rc_car_bot_passive)
