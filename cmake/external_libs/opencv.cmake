# Try looking for OpenCV in 3 ways:
#   1) Look for CMake package
#   2) Look for a pkg-config package called opencv4
#   3) Look for a pkg-config package called opencv
#
# Hopefully this should work with all platforms/versions of OpenCV.
if(NOT OpenCV_FOUND)
    find_package(OpenCV QUIET)
    if (OpenCV_FOUND)

    else()
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
