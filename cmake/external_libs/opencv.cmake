# Using the OpenCV CMake package has broken the build in a couple of cases in
# the past (on macOS and once when built from source on Ubuntu), because it
# sometimes (with old versions of CMake, maybe?) unconditionally sets the C++
# version to C++11, even though we explicitly specify that we need C++14.
#
# So we try looking for OpenCV in 3 ways:
#   1) Look for a pkg-config package called opencv4
#   2) Look for a pkg-config package called opencv
#   3) Look for CMake package
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(OpenCV QUIET opencv4)
    if (NOT OpenCV_FOUND)
        pkg_check_modules(OpenCV QUIET opencv)
        if (NOT OpenCV_FOUND)
            message(FATAL_ERROR "Could not find OpenCV")
        endif()
    endif()

    message("-- Found OpenCV v${OpenCV_VERSION}")
    BoB_add_include_directories(${OpenCV_INCLUDE_DIRS})
    BoB_add_link_libraries(${OpenCV_LIBS} ${OpenCV_LINK_LIBRARIES})
    BoB_add_link_directories(${OpenCV_LIB_DIR})
else()
    BoB_find_package(OpenCV)
endif()
