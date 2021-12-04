cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(snapshot_bot)

set(IS_EXPERIMENT TRUE)

macro(add_option VARNAME DESCRIPTION)
    if(NOT DEFINED ${VARNAME})
        set(${VARNAME} OFF CACHE BOOL "${DESCRIPTION}")
    else()
        set(${VARNAME} "${${VARNAME}}" CACHE BOOL "${DESCRIPTION}")
    endif()

    if(${VARNAME})
        add_definitions(-D${VARNAME})
    endif()

    message(STATUS "${VARNAME}: ${${VARNAME}}")
endmacro()

add_option(USE_ODK "Build with support for ODK camera")
add_option(USE_VICON "Build with support for Vicon tracking")

set(MODULES hid imgproc navigation robots video)
if(USE_ODK)
    list(APPEND MODULES video/odk2)
endif()
if(USE_VICON)
    list(APPEND MODULES vicon)
endif()

find_package(BoBRobotics QUIET REQUIRED COMPONENTS ${MODULES})
add_library(snapshot_bot_common OBJECT common.cc config.cc image_input.cc memory.cc)
target_link_libraries(snapshot_bot_common PUBLIC BoBRobotics::imgproc
                      BoBRobotics::navigation BoBRobotics::video)

add_executable(snapshot_bot snapshot_bot.cc)
target_link_libraries(snapshot_bot PRIVATE snapshot_bot_common
                      ${BoBRobotics_LIBRARIES})

# Put the default config into a file for the user to edit
add_custom_command(TARGET snapshot_bot
                   POST_BUILD
                   COMMAND "${CMAKE_CURRENT_BINARY_DIR}/snapshot_bot" --dump-config)

add_executable(offline_train offline_train.cc)
target_link_libraries(offline_train PRIVATE snapshot_bot_common)

configure_file(mask.png "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(segmentation.png "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(config_demo.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

find_package(BoBThirdParty COMPONENTS matplotlibcpp)
if(NOT BoBThirdParty_FOUND)
    message(WARNING "Could not find Python; will not build extra tools")
else()
    add_executable(live_difference live_difference.cc)
    target_link_libraries(live_difference PRIVATE ${BoBThirdParty_LIBRARIES}
                          snapshot_bot_common)
endif()
