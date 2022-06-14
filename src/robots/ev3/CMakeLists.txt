cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(robots/ev3)

find_package(BoBRobotics REQUIRED COMPONENTS common net)
BoB_module(ev3.cc mindstorms_imu.cc)

find_package(BoBThirdParty REQUIRED COMPONENTS ev3dev-lang-cpp)
target_compile_definitions(${BOB_MODULE_TARGET} PUBLIC ${BoBThirdParty_DEFINITIONS})
target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${BoBThirdParty_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${BoBThirdParty_LIBRARIES})
