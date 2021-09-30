# Using the OpenCV CMake package has broken the build in a couple of cases in
# the past (on macOS and once when built from source on Ubuntu), because it
# sometimes (with old versions of CMake, maybe?) unconditionally sets the C++
# version to C++11, even though we explicitly specify that we need C++14. On the
# other hand, using pkg-config doesn't always find the contrib libs seemingly
# (on JK's machine). Also, the CMake package isn't always present so we can't
# rely on it. What a mess.
#
# By default we try looking for OpenCV in 3 ways:
#   1) Look for CMake package
#   2) Look for a pkg-config package called opencv4
#   3) Look for a pkg-config package called opencv
#
# We also provide an extra option to explicitly use pkg-config.
if(NOT OpenCV_FOUND)
    # The CMake package file for OpenCV v4.3+ appears to have a bug in which
    # means that the C++ standard is forced to C++11, which will break BoB
    # robotics code as we use C++14 features all over the place. Setting this
    # variable seems to be a good workaround.
    set(OPENCV_COMPILE_FEATURES "")

    if(NOT FIND_OPENCV_WITH_PKGCONFIG)
        find_package(OpenCV QUIET)
    endif()
    if (NOT OpenCV_FOUND)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(OpenCV QUIET opencv4)
        if (NOT OpenCV_FOUND)
            pkg_check_modules(OpenCV QUIET opencv)
            if (NOT OpenCV_FOUND)
                message(FATAL_ERROR "Could not find OpenCV")
            endif()
        endif()
    endif()

    message("-- Found OpenCV v${OpenCV_VERSION}")
    BoB_add_include_directories(${OpenCV_INCLUDE_DIRS})
    BoB_add_link_libraries(${OpenCV_LIBS} ${OpenCV_LINK_LIBRARIES})
    BoB_add_link_directories(${OpenCV_LIB_DIR})
endif()
