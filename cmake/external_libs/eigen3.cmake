if(UNIX)
    BoB_add_pkg_config_libraries(eigen3)
else()
    if(NOT TARGET Eigen3::Eigen)
        message(FATAL_ERROR "Eigen 3 not found")
    endif()
    BoB_add_link_libraries(Eigen3::Eigen)
endif()

# For CMake < 3.9, we need to make the target ourselves
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_flags(-fopenmp)
elseif(NOT OpenMP_CXX_FOUND)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})

    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

    BoB_add_link_libraries(OpenMP::OpenMP_CXX)
endif()
