cmake_minimum_required(VERSION 3.10)
project(snapshot_bot)

set(IS_EXPERIMENT TRUE)
include(../../cmake/BoBRobotics.cmake)

set(MODULES hid imgproc navigation robots vicon video)
if(USE_ODK)
    list(APPEND MODULES video/odk2)
    add_definitions(-DUSE_ODK)
endif()
find_package(BoBRobotics QUIET REQUIRED COMPONENTS ${MODULES})

add_executable(snapshot_bot snapshot_bot.cc memory.cc image_input.cc)
target_link_libraries(snapshot_bot PUBLIC ${BoBRobotics_LIBRARIES})

add_executable(offline_train offline_train.cc memory.cc image_input.cc)
target_link_libraries(offline_train PUBLIC BoBRobotics::imgproc
                      BoBRobotics::navigation BoBRobotics::video)
