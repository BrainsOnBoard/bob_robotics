cmake_minimum_required(VERSION 3.10)
project(imgui_example)

# Note that we don't actually use any BoB robotics core components here, just
# third-party libraries
include(../../cmake/BoBRobotics.cmake)
find_package(BoBThirdParty REQUIRED COMPONENTS imgui plog)
add_executable(${PROJECT_NAME} imgui.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC ${BoBThirdParty_LIBRARIES})
