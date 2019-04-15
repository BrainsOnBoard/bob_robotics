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

set(AR_STAGING_PATH $ENV{ARSDK_ROOT}/out/arsdk-native/staging)
include_directories(${AR_STAGING_PATH}/include)
set(AR_LIB_PATH ${AR_STAGING_PATH}/lib)
link_directories($ENV{ARSDK_ROOT}/out/arsdk-native/staging/lib)
target_link_libraries(bebop_test PUBLIC ${OpenCV_LIBS}
                                        ${AVCODEC_LIB}
                                        ${AVFORMAT_LIB}
                                        ${AVUTIL_LIB}
                                        ${SWSCALE_LIB}
                                        ${AR_LIB_PATH}/libarsal.so
                                        ${AR_LIB_PATH}/libardiscovery.so
                                        ${AR_LIB_PATH}/libarcontroller.so
                                        ${AR_LIB_PATH}/libarnetworkal.so
                                        ${AR_LIB_PATH}/libarcommands.so
                                        ${AR_LIB_PATH}/libmux.so
                                        ${AR_LIB_PATH}/libpomp.so
                                        ${AR_LIB_PATH}/libjson-c.so.2
                                        ${AR_LIB_PATH}/libarstream.so
                                        ${AR_LIB_PATH}/libarstream2.so
                                        ${AR_LIB_PATH}/libarnetwork.so
                                        ${AR_LIB_PATH}/librtsp.so
                                        ${AR_LIB_PATH}/libsdp.so
                                        ${AR_LIB_PATH}/libulog.so
                                        ${AR_LIB_PATH}/libarmedia.so
                                        ${AR_LIB_PATH}/libfutils.so
                                        $ENV{BOB_LIBS}
                                        ${CMAKE_THREAD_LIBS_INIT})
