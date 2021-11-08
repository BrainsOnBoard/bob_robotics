cmake_minimum_required(VERSION 3.10)
project(viz/car_display)

find_package(BoBRobotics REQUIRED COMPONENTS common)
find_package(SDL2 REQUIRED)

BoB_module(car_display.cc)
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${SDL2_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${SDL2_LIBRARIES})
