cmake_minimum_required(VERSION 3.1)

macro(add_compile_flags EXTRA_ARGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(add_linker_flags EXTRA_ARGS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(BoB_parse_arguments)
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSED_ARGS
                          ""
                          ""
                          "BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS"
                          "${ARGV}")

    # # Replace slashes with underscores (used in BoB modules, e.g. robots/bebop)
    # string(REPLACE / _ PARSED_ARGS_NAME ${PARSED_ARGS_NAME})
    # project(${PARSED_ARGS_NAME})
endmacro()

macro(base_packages)
    if(NOT TARGET Eigen3::Eigen)
        find_package(Eigen3)
    endif()
    if(NOT TARGET OpenMP::OpenMP_CXX)
        find_package(OpenMP)
    endif()
    if(NOT TARGET GLEW::GLEW)
        find_package(GLEW)
    endif()
endmacro()

macro(BoB_module NAME)
    set(BOB_TARGETS bob_${NAME})
    string(REPLACE / _ BOB_TARGETS ${BOB_TARGETS})
    project(${BOB_TARGETS})

    file(GLOB SRC_FILES
        "${BOB_ROBOTICS_PATH}/include/${NAME}/*.h"
        "*.cc"
    )
    add_library(${BOB_TARGETS} STATIC ${SRC_FILES})
    set_target_properties(${BOB_TARGETS} PROPERTIES PREFIX ./lib)
    add_compile_definitions(NO_HEADER_DEFINITIONS)

    base_packages()
endmacro()

macro(BoB_project)
    BoB_parse_arguments(${ARGN})

    # Use current folder as project name
    get_filename_component(NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
    project(${NAME})

    # Build each *.cc file as a separate executable
    file(GLOB CC_FILES "*.cc")
    file(GLOB H_FILES "*.h")
    foreach(file IN LISTS CC_FILES)
        get_filename_component(shortname ${file} NAME)
        string(REGEX REPLACE "\\.[^.]*$" "" target ${shortname})
        add_executable(${target} "${file}" "${H_FILES}")
        set(BOB_TARGETS "${BOB_TARGETS};${target}")
    endforeach()

    base_packages()
    BoB_platforms(${PARSED_ARGS_PLATFORMS})
    BoB_modules(${PARSED_ARGS_BOB_MODULES})
    BoB_external_libraries(${PARSED_ARGS_EXTERNAL_LIBS})
    BoB_third_party(${PARSED_ARGS_THIRD_PARTY})

    # BoB robotics libs are output here
    link_directories(${BOB_ROBOTICS_PATH}/lib)

    # Link threading lib
    BoB_add_link_libraries(${CMAKE_THREAD_LIBS_INIT})

    # Copy all DLLs over from vcpkg dir
    if(WIN32)
        file(GLOB dll_files "$ENV{VCPKG_ROOT}/installed/${CMAKE_GENERATOR_PLATFORM}-windows/bin/*.dll")
        foreach(file IN LISTS dll_files)
            get_filename_component(filename "${file}" NAME)
            if(NOT EXISTS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${filename}")
                message("Copying ${filename} to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}...")
                file(COPY "${file}"
                     DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
            endif()
        endforeach()
    endif()
endmacro()

function(BoB_platforms)
    # If it's an empty list, return
    if(${ARGC} EQUAL 0)
        return()
    endif()

    # Check the platforms are valid
    foreach(platform IN LISTS ARGV)
        if(NOT "${platform}" STREQUAL unix AND NOT "${platform}" STREQUAL linux AND NOT "${platform}" STREQUAL windows AND NOT "${platform}" STREQUAL all)
            message(FATAL_ERROR "Bad platform: ${platform}. Possible platforms are unix, linux, windows or all.")
        endif()
    endforeach()

    # Check for platform all
    list(FIND ARGV all index)
    if(${index} GREATER -1)
        return()
    endif()

    # Check for platform unix
    if(UNIX)
        list(FIND ARGV unix index)
        if(${index} GREATER -1)
            return()
        endif()

        # Check for platform linux
        if("${CMAKE_SYSTEM_NAME}" STREQUAL Linux)
            list(FIND ARGV linux index)
            if(${index} GREATER -1)
                return()
            endif()
        endif()
    endif()

    # Check for platform windows
    if(WIN32)
        list(FIND ARGV windows index)
        if(${index} GREATER -1)
            return()
        endif()
    endif()

    # Otherwise we haven't matched any platforms
    message(FATAL_ERROR "This machine is not one of the supported platforms for this project (${ARGV}).")
endfunction()

macro(BoB_add_link_libraries)
    foreach(target IN LISTS BOB_TARGETS)
        if(NOT "${target}" STREQUAL "")
            target_link_libraries(${target} ${ARGV})
        endif()
    endforeach()

    set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${ARGV}"
        CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_LIBRARIES)
endmacro()

function(BoB_add_include_directories)
    # Include directory locally...
    include_directories(${ARGV})

    # ...and export to parent project
    set(${PROJECT_NAME}_INCLUDE_DIRS "${${PROJECT_NAME}_INCLUDE_DIRS};${ARGV}"
        CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_INCLUDE_DIRS)
endfunction()

function(BoB_modules)
    foreach(module IN LISTS ARGV)
        set(module_path ${BOB_ROBOTICS_PATH}/src/${module})
        string(REPLACE / _ module_name ${module})
        foreach(target IN LISTS BOB_TARGETS)
            if(NOT "${target}" STREQUAL "")
                add_dependencies(${target} bob_${module_name})
            endif()
        endforeach()

        if(NOT TARGET bob_${module_name})
            add_subdirectory(${module_path} "${CMAKE_CURRENT_BINARY_DIR}/bob_modules/${module_name}")
        endif()

        BoB_add_include_directories(${bob_${module_name}_INCLUDE_DIRS})
        set(LIB_PATH "${BOB_ROBOTICS_PATH}/lib/libbob_${module_name}${CMAKE_STATIC_LIBRARY_SUFFIX}")
        BoB_add_link_libraries(${bob_${module_name}_LIBRARIES} ${LIB_PATH})
    endforeach()
endfunction()

function(BoB_find_package)
    find_package(${ARGV})
    BoB_add_include_directories(${${ARGV0}_INCLUDE_DIRS})
    BoB_add_link_libraries(${${ARGV0}_LIBS} ${${ARGV0}_LIBRARIES})
endfunction()

function(BoB_add_pkg_config_libraries)
    find_package(PkgConfig)
    foreach(lib IN LISTS ARGV)
        pkg_check_modules(${lib} REQUIRED ${lib})
        BoB_add_include_directories(${${lib}_INCLUDE_DIRS})
        BoB_add_link_libraries(${${lib}_LIBRARIES})
    endforeach()
endfunction()

function(BoB_external_libraries)
    foreach(lib IN LISTS ARGV)
        if(${lib} STREQUAL i2c)
            # Special handling for i2c: only link against lib for new version of i2c-tools
            execute_process(COMMAND "${BOB_ROBOTICS_PATH}/make_common/is_i2c_tools_new.sh"
                            RESULT_VARIABLE rv)
            if(${rv} STREQUAL 0)
                BoB_add_link_libraries("i2c")
            endif()
        elseif(${lib} STREQUAL opencv)
            BoB_find_package(OpenCV REQUIRED)
        elseif(${lib} STREQUAL eigen3)
            if(NOT TARGET Eigen3::Eigen)
                message(FATAL_ERROR "Eigen 3 not found")
            endif()
            BoB_add_link_libraries(Eigen3::Eigen)

            # For CMake < 3.9, we need to make the target ourselves
            if(NOT OpenMP_CXX_FOUND)
                find_package(Threads REQUIRED)
                add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
                set_property(TARGET OpenMP::OpenMP_CXX
                             PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})

                # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
                set_property(TARGET OpenMP::OpenMP_CXX
                             PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

                BoB_add_link_libraries(OpenMP::OpenMP_CXX)
            endif()
        elseif(${lib} STREQUAL sfml-graphics)
            if(UNIX)
                BoB_add_pkg_config_libraries(sfml-graphics)
            else()
                find_package(SFML REQUIRED graphics)
                BoB_add_include_directories(${SFML_INCLUDE_DIR})
                BoB_add_link_libraries(${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
            endif()
        elseif(${lib} STREQUAL sdl2)
            if(UNIX)
                BoB_add_pkg_config_libraries(sdl2)
            else()
                find_package(SDL2 REQUIRED)
                BoB_add_include_directories(${SDL2_INCLUDE_DIRS})
                BoB_add_link_libraries(${SDL2_LIBRARIES})
            endif()
        elseif(${lib} STREQUAL glfw3)
            find_package(glfw3 REQUIRED)
            BoB_add_link_libraries(glfw)
            BoB_external_libraries(opengl)
        elseif(${lib} STREQUAL glew)
            if(NOT TARGET GLEW::GLEW)
                message(FATAL_ERROR "Could not find glew")
            endif()

            BoB_add_link_libraries(GLEW::GLEW)
            BoB_external_libraries(opengl)
        elseif(${lib} STREQUAL opengl)
            # Newer versions of cmake give a deprecation warning
            set(OpenGL_GL_PREFERENCE LEGACY)

            find_package(OpenGL REQUIRED)
            BoB_add_include_directories(${OPENGL_INCLUDE_DIR})
            BoB_add_link_libraries(${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
        else()
            message(FATAL_ERROR "${lib} is not a recognised library name")
        endif()
    endforeach()
endfunction()

macro(exec_or_fail)
    execute_process(COMMAND ${ARGV} RESULT_VARIABLE rv OUTPUT_VARIABLE SHELL_OUTPUT)
    if(NOT ${rv} EQUAL 0)
        message(FATAL_ERROR "Error while executing: ${ARGV}")
    endif()
endmacro()

function(BoB_third_party)
    foreach(module IN LISTS ARGV)
        if("${module}" STREQUAL matplotlibcpp)
            find_package(PythonLibs REQUIRED)
            BoB_add_include_directories(${PYTHON_INCLUDE_DIRS})
            BoB_add_link_libraries(${PYTHON_LIBRARIES})

            # Also include numpy headers on *nix (gives better performance)
            if(WIN32)
                add_compile_definitions(WITHOUT_NUMPY)
            else()
                exec_or_fail("python" "${BOB_ROBOTICS_PATH}/make_common/find_numpy.py")
                BoB_add_include_directories(${SHELL_OUTPUT})
            endif()
        else()
            # Checkout git submodules under this path
            find_package(Git REQUIRED)
            exec_or_fail(${GIT_EXECUTABLE} submodule update --init --recursive third_party/${module}
                         WORKING_DIRECTORY ${BOB_ROBOTICS_PATH})

            # If this folder is a cmake project, then build it
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            if(EXISTS ${module_path}/CMakeLists.txt)
                add_subdirectory(${module_path} "${CMAKE_CURRENT_BINARY_DIR}/bob_third_party/${module}")
            endif()

            # Add to include path
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            include_directories(${module_path} ${module_path}/include)

            # Extra actions
            if(${module} STREQUAL ev3dev-lang-cpp)
                BoB_add_link_libraries(${BOB_ROBOTICS_PATH}/lib/libev3dev${CMAKE_STATIC_LIBRARY_SUFFIX})
            endif()
        endif()
    endforeach()
endfunction()

# Don't allow in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.
    Please make a new directory (called a build directory) and run CMake from there.
    You may need to remove CMakeCache.txt." )
endif()

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Flags for gcc
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    add_compile_flags("-Wall -Wpedantic -Wextra")
endif()

# Set output directories for libs and executables
set(BOB_ROBOTICS_PATH "${CMAKE_CURRENT_LIST_DIR}/..")
if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BOB_ROBOTICS_PATH}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BOB_ROBOTICS_PATH}/bin)
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BOB_ROBOTICS_PATH}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BOB_ROBOTICS_PATH}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${BOB_ROBOTICS_PATH}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${BOB_ROBOTICS_PATH}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${BOB_ROBOTICS_PATH}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${BOB_ROBOTICS_PATH}/lib")

# Use vcpkg on Windows
if(WIN32)
    # Use vcpkg's cmake toolchain
    if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "")
    endif()

    # The vcpkg toolchain in theory should do something like this already, but
    # if we don't do this, then cmake can't find any of vcpkg's packages
    file(GLOB children "$ENV{VCPKG_ROOT}/installed/${CMAKE_GENERATOR_PLATFORM}-windows")
    foreach(child IN LISTS children)
        if(IS_DIRECTORY "${child}")
            list(APPEND CMAKE_PREFIX_PATH "${child}")
        endif()
    endforeach()

    # Suppress warnings about std::getenv being insecure
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# Assume we always need plog
BoB_third_party(plog)

# Assume we always want threading
find_package(Threads REQUIRED)

# Default include paths
include_directories(${BOB_ROBOTICS_PATH}
                    ${BOB_ROBOTICS_PATH}/include)

# Disable some of the units types in units.h for faster compilation
add_compile_definitions(
    DISABLE_PREDEFINED_UNITS
    ENABLE_PREDEFINED_LENGTH_UNITS
    ENABLE_PREDEFINED_TIME_UNITS
    ENABLE_PREDEFINED_ANGLE_UNITS
    ENABLE_PREDEFINED_VELOCITY_UNITS
    ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS
)
