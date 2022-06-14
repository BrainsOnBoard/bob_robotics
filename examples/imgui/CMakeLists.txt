cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(imgui_example)

# Note that we don't actually use any BoB robotics core components here, just
# third-party libraries
find_package(BoBThirdParty REQUIRED COMPONENTS imgui plog)
add_executable(${PROJECT_NAME} imgui.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC ${BoBThirdParty_LIBRARIES})
