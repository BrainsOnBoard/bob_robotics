cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(simulated_ackerman_car)
BoB_modules(robots viz/car_display)

target_link_libraries(simulated_ackerman_car PUBLIC $ENV{BOB_LINK_LIBS})