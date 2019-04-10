cmake_minimum_required(VERSION 2.8)

set(BOB_ROBOTICS_PATH "${CMAKE_CURRENT_LIST_DIR}/..")
include_directories(${BOB_ROBOTICS_PATH}/include
                    ${BOB_ROBOTICS_PATH}/third_party/plog/include)

link_directories(${BOB_ROBOTICS_PATH}/lib)
