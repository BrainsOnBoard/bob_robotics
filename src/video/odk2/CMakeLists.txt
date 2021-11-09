cmake_minimum_required(VERSION 3.10)
project(video/odk2)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS video)
find_package(BoBThirdParty REQUIRED COMPONENTS development_kit_odk2)

BoB_module(odk2.cc)
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${BoBRobotics_LIBRARIES}
                      ${BoBThirdParty_LIBRARIES})
