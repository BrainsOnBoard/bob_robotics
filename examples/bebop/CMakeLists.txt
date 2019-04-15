cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_project(bebop_test)
BoB_modules(robots hid bebop video imgproc common)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

find_library(AVCODEC_LIB avcodec REQUIRED)
find_library(AVFORMAT_LIB avformat REQUIRED)
find_library(AVUTIL_LIB avutil REQUIRED)
find_library(SWSCALE_LIB swscale REQUIRED)

include_directories($ENV{BOB_INCLUDE_DIRS})
target_link_libraries(bebop_test PUBLIC ${OpenCV_LIBS}
                                        ${AVCODEC_LIB}
                                        ${AVFORMAT_LIB}
                                        ${AVUTIL_LIB}
                                        ${SWSCALE_LIB}
                                        $ENV{BOB_LINK_LIBS}
                                        ${CMAKE_THREAD_LIBS_INIT})
