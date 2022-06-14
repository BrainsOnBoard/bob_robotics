# The video decoding relies on FFMPEG libraries
find_package(FFMPEG COMPONENTS avcodec avformat avutil swscale)
set(ARSDK_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIRS})
set(ARSDK_LIBRARIES ${FFMPEG_LIBRARIES})

# We need the user to tell us where ARSDK is installed
if(FFMPEG_FOUND AND DEFINED ENV{ARSDK_ROOT_DIR} AND EXISTS "$ENV{ARSDK_ROOT_DIR}")
    set(ARSDK_ROOT_DIR "$ENV{ARSDK_ROOT_DIR}")
    set(_ARSDK_STAGING "${ARSDK_ROOT_DIR}/out/arsdk-native/staging")
    set(ARSDK_LIBRARY_DIRS "${_ARSDK_STAGING}/usr/lib")
    set(ARSDK_INCLUDE_DIRS "${_ARSDK_STAGING}/usr/include")

    foreach(LIB arsal ardiscovery arcontroller arnetworkal arcommands mux pomp
            json-c arstream arstream2 arnetwork rtsp sdp ulog armedia futils)
        list(APPEND ARSDK_LIBRARIES
             "${ARSDK_LIBRARY_DIRS}/${CMAKE_SHARED_LIBRARY_PREFIX}${LIB}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    endforeach()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ARSDK REQUIRED_VARS ARSDK_ROOT_DIR)
