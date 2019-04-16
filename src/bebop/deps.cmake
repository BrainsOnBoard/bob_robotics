# Needed for video
BoB_external_libraries(libavcodec libavformat libavutil libswscale)

set(AR_STAGING_PATH $ENV{ARSDK_ROOT}/out/arsdk-native/staging)
set(AR_LIB_PATH ${AR_STAGING_PATH}/lib)
BoB_add_include_directories(${AR_STAGING_PATH}/include)
BoB_add_link_libraries(${AR_LIB_PATH}/libarsal.so
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
                       ${AR_LIB_PATH}/libfutils.so)
