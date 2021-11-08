cmake_minimum_required(VERSION 3.10)
project(robot_net_example)

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS hid net os robots robots/tank/net video)

foreach(TARGET robot computer)
    add_executable(${TARGET} ${TARGET}.cc)
    target_link_libraries(${TARGET} PUBLIC ${BoBRobotics_LIBRARIES})
endforeach(TARGET robot computer)
