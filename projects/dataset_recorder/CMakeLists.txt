cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(dataset_recorder)

set(IS_EXPERIMENT TRUE)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS gps navigation
             robots/ackermann robots/control video)
find_package(BoBThirdParty COMPONENTS matplotlibcpp)

add_executable(dataset_recorder dataset_recorder.cc)
target_link_libraries(dataset_recorder PUBLIC BoBRobotics::gps
                      BoBRobotics::navigation BoBRobotics::robots__ackermann
                      BoBRobotics::video)
if(DUMMY_CAMERA)
    target_compile_definitions(dataset_recorder PUBLIC DUMMY_CAMERA)
endif()

add_executable(auto_dataset_recorder auto_dataset_recorder.cc)
target_link_libraries(auto_dataset_recorder PUBLIC ${BoBRobotics_LIBRARIES})
if(DUMMY_CAMERA)
    target_compile_definitions(auto_dataset_recorder PUBLIC DUMMY_CAMERA)
endif()

if(BoBThirdParty_FOUND)
    add_executable(auto_dataset_recorder_simulated auto_dataset_recorder_simulated.cc)
    target_link_libraries(auto_dataset_recorder_simulated PUBLIC
                          BoBRobotics::navigation BoBRobotics::robots__ackermann
                          BoBRobotics::robots__control BoBRobotics::video
                          ${BoBThirdParty_LIBRARIES})
    if(DUMMY_CAMERA)
        target_compile_definitions(auto_dataset_recorder PUBLIC DUMMY_CAMERA)
    endif()
endif()
