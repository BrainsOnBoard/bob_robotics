cmake_minimum_required(VERSION 2.8)
project(msp)

add_definitions(-DASIO_STANDALONE)

if(WIN32)
    include_directories(${ASIO_HEADER_PATH})
endif()

find_package(Threads)

set(MSP_SOURCE_DIR msp/src)
set(MSP_INCLUDE_DIR msp/inc/msp)
include_directories(${MSP_INCLUDE_DIR})

################################################################################
### libraries

# low-level API
add_library(msp ${MSP_SOURCE_DIR}/MSP.cpp)
target_link_libraries(msp ${CMAKE_THREAD_LIBS_INIT})

# client library
add_library(mspclient ${MSP_SOURCE_DIR}/Client.cpp)
target_link_libraries(mspclient ${CMAKE_THREAD_LIBS_INIT})

# high-level API
add_library(msp_fcu ${MSP_SOURCE_DIR}/FlightController.cpp)
target_link_libraries(msp_fcu mspclient)
