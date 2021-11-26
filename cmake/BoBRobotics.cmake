function(make_base_target)
    if(CMAKE_BUILD_TYPE)
        string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
    else()
        # Default to doing a release build
        set(CMAKE_BUILD_TYPE RELEASE)
    endif()
    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

    add_library(bob_base INTERFACE)
    add_library(BoBRobotics::base ALIAS bob_base)

    target_include_directories(bob_base INTERFACE
                               "${BOB_ROBOTICS_PATH}/include"
                               "${BOB_ROBOTICS_PATH}")

    # Define DEBUG macro
    target_compile_definitions(bob_base INTERFACE "$<$<CONFIG:DEBUG>:DEBUG>")

    if(NOT MSVC)
        # Let user explicitly disable ccache if desired (e.g. for comparing
        # compile times)
        if(NOT DEFINED USE_CCACHE OR USE_CCACHE)
            # Use ccache if present to speed up repeat builds
            find_program(CCACHE ccache)
            if(CCACHE)
                set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
                set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
            else()
                message(WARNING "ccache not found. Install for faster repeat builds.")
            endif()
        endif()

        # Irritatingly, neither GCC nor Clang produce nice ANSI-coloured output if they detect
        # that output "isn't a terminal" - which seems to include whatever pipe-magick cmake includes.
        # https://medium.com/@alasher/colored-c-compiler-output-with-ninja-clang-gcc-10bfe7f2b949
        #
        # When compiling via Jenkins though, we don't want colourised output because
        # a) the Jenkins logs don't support colours anyway and b) the colour escape
        # sequences seem to break Jenkins' auto-parsing of error messages.
        if(NOT "$ENV{USER}" STREQUAL jenkins)
            if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
                target_compile_options(bob_base INTERFACE -fdiagnostics-color=always)
            elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
                target_compile_options(bob_base INTERFACE -fcolor-diagnostics)
            endif()
        endif()

        # Default warnings to enable
        set(DEFAULT_COMPILER_FLAGS -Wall -Wpedantic -Wextra)

        # Gcc has an annoying feature where you can mark functions with
        # __attribute__((warn_unused_result)) and then the calling code *has*
        # to do something with the result and can't ignore it; hacks such as
        # (void) annoyingFunction() don't work either. We're mostly
        # seeing this warning for calls to std::system() (in our code and third-
        # party code), but in those cases we generally really don't care about
        # the return value. So let's just disable it globally to save faffing
        # around.
        list(APPEND DEFAULT_COMPILER_FLAGS -Wno-unused-result)

        # Default to building with -march=native, if supported
        if(NOT DEFINED BUILD_MARCH_NATIVE OR BUILD_MARCH_NATIVE)
            list(APPEND DEFAULT_COMPILER_FLAGS -march=native)
        endif()

        include(CheckCXXCompilerFlag)
        foreach(FLAG IN LISTS DEFAULT_COMPILER_FLAGS)
            check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORTS_${FLAG})
            if(COMPILER_SUPPORTS_${FLAG})
                target_compile_options(bob_base INTERFACE ${FLAG})
            endif()
        endforeach(FLAG IN LISTS DEFAULT_COMPILER_FLAGS)
    endif(NOT MSVC)

    if(WIN32)
        # Suppress warnings about std::getenv being insecure
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)

        # We don't want the min/max macros defined in windows.h
        add_definitions(-DNOMINMAX)

        # <cmath> doesn't normally define M_PI etc on windows
        add_definitions(-D_USE_MATH_DEFINES)

        # the version of the Windows API that we want
        add_definitions(-D_WIN32_WINNT=_WIN32_WINNT_WIN7)

        # for a less bloated version of windows.h
        add_definitions(-D_WIN32_LEAN_AND_MEAN)

        # disable the winsock v1 API, which is included by default and conflicts
        # with v2 of the API
        add_definitions(-D_WINSOCKAPI_)
    endif(WIN32)
endfunction(make_base_target)

if(WIN32 AND NOT CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
    message(STATUS "Found vcpkg at $ENV{VCPKG_ROOT}")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}\\scripts\\buildsystems\\vcpkg.cmake" CACHE PATH "toolchain file")
endif()

# Look for additional CMake packages in the current folder
if(NOT "${CMAKE_CURRENT_LIST_DIR}" IN_LIST CMAKE_MODULE_PATH)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
endif()

# Build with C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
set(CXX_STANDARD_REQUIRED TRUE)

get_filename_component(BOB_ROBOTICS_PATH .. ABSOLUTE BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
