cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(stone_cx)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS common hid imgproc net os robots video)
find_package(GeNN REQUIRED)
find_package(OpenCV REQUIRED)

# build simulator
add_genn_model(model model.cc BACKEND CPU SHARED_LIBRARY_MODEL OFF)

# NB: simulatorReplay is not included as it won't compile with GeNN 4
add_executable(stone_cx_simulator simulator.cc spline.cc visualizationCommon.cc)
target_include_directories(stone_cx_simulator PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stone_cx_simulator PUBLIC BoBRobotics::common model
                      ${OpenCV_LIBS})

add_executable(stone_cx_robot robot.cc visualizationCommon.cc)
target_include_directories(stone_cx_robot PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stone_cx_robot PUBLIC ${BoBRobotics_LIBRARIES} model
                      ${OpenCV_LIBS})
