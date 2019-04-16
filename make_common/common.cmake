cmake_minimum_required(VERSION 3.1)

macro(ADD_CXXFLAGS EXTRA_ARGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(ADD_LDFLAGS EXTRA_ARGS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(BoB_parse_arguments)
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSED_ARGS "" "NAME" "BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY" "${ARGV}")

    # Need to give a name for the project
    if(NOT PARSED_ARGS_NAME)
        message(FATAL_ERROR "You must provide a name")
    endif(NOT PARSED_ARGS_NAME)

    # Replace slashes with underscores (used in BoB modules, e.g. robots/bebop)
    string(REPLACE / _ PARSED_ARGS_NAME ${PARSED_ARGS_NAME})
    project(${PARSED_ARGS_NAME})
endmacro()

function(BoB_module NAME)
    set(MODULE_NAME bob_${NAME})
    string(REPLACE / _ MODULE_NAME ${MODULE_NAME})
    project(${MODULE_NAME})
    BoB_include_module_deps(${NAME})

    file(GLOB SRC_FILES
        "${BOB_ROBOTICS_PATH}/include/${NAME}/*.h"
        "*.cc"
    )
    add_library(${MODULE_NAME} STATIC ${SRC_FILES})
endfunction()

macro(BoB_project)
    BoB_parse_arguments(${ARGN})

    file(GLOB SRC_FILES
        "*.cc"
    )
    foreach(file IN LISTS SRC_FILES)
        get_filename_component(shortname ${file} NAME)
        string(REGEX REPLACE "\\.[^.]*$" "" file_without_ext ${shortname})
        add_executable(${file_without_ext} ${file})
        set(BOB_TARGETS "${BOB_TARGETS};${file_without_ext}")
    endforeach()

    BoB_modules(${PARSED_ARGS_BOB_MODULES})
    BoB_external_libraries(${PARSED_ARGS_EXTERNAL_LIBS})
    BoB_third_party(${PARSED_ARGS_THIRD_PARTY})
    BoB_build()
endmacro()

macro(BoB_add_link_libraries)
    set(ENV{BOB_LINK_LIBS} "${ARGV};$ENV{BOB_LINK_LIBS}")
endmacro()

function(BoB_add_include_directories)
    # Include directory locally...
    include_directories(${ARGV})

    # ...and export to parent project
    set(ENV{BOB_INCLUDE_DIRS} "$ENV{BOB_INCLUDE_DIRS};${ARGV}")
endfunction()

function(BoB_include_module_deps MODULE)
    set(DEPS_PATH ${BOB_ROBOTICS_PATH}/src/${MODULE}/deps.cmake)
    if(EXISTS "${DEPS_PATH}")
        include("${DEPS_PATH}")
    endif()
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

        set(LIB_PATH "${BOB_ROBOTICS_PATH}/lib/libbob_${module_name}.a")
        if(NOT "$ENV{BOB_LINK_LIBS}" MATCHES ${LIB_PATH})
            add_subdirectory(${module_path} ${module_path}/build)
            BoB_add_link_libraries(${LIB_PATH})
        endif()

        BoB_include_module_deps(${module})
    endforeach()
endfunction()

function(BoB_external_libraries)
    foreach(lib IN LISTS ARGV)
        # Special handling for i2c: no pkg-config + only link against lib for new version
        if(${lib} STREQUAL i2c)
            execute_process(COMMAND "${BOB_ROBOTICS_PATH}/make_common/is_i2c_tools_new.sh"
                            RESULT_VARIABLE rv)
            if(${rv} STREQUAL 0)
                BoB_add_link_libraries("i2c")
            endif()
        else()
            pkg_check_modules(${lib} REQUIRED ${lib})
            BoB_add_include_directories(${${lib}_INCLUDE_DIRS})
            BoB_add_link_libraries(${${lib}_LIBRARIES})
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

            exec_or_fail("python" "${BOB_ROBOTICS_PATH}/make_common/find_numpy.py")
            BoB_add_include_directories(${SHELL_OUTPUT})
        else()
            # Checkout git submodules under this path
            find_package(Git REQUIRED)
            exec_or_fail(${GIT_EXECUTABLE} submodule update --init --recursive third_party/${module}
                         WORKING_DIRECTORY ${BOB_ROBOTICS_PATH})

            # If this folder is a cmake project, then build it
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            if(EXISTS ${module_path}/CMakeLists.txt)
                add_subdirectory(${module_path} ${module_path}/build)
            endif()

            # Add to include path
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            include_directories(${module_path} ${module_path}/include)

            # Extra actions
            if(${module} STREQUAL ev3dev-lang-cpp)
                BoB_add_link_libraries(${BOB_ROBOTICS_PATH}/lib/libev3dev.a)
            endif()
        endif()
    endforeach()
endfunction()

function(BoB_build)
    # Use C++14
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # Flags for gcc
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        ADD_CXXFLAGS("-Wall -Wpedantic -Wextra")
    endif()

    # BoB robotics libs are output here
    link_directories(${BOB_ROBOTICS_PATH}/lib)

    # Disable some of the units types in units.h for faster compilation
    add_compile_definitions(
        DISABLE_PREDEFINED_UNITS
        ENABLE_PREDEFINED_LENGTH_UNITS
        ENABLE_PREDEFINED_TIME_UNITS
        ENABLE_PREDEFINED_ANGLE_UNITS
        ENABLE_PREDEFINED_VELOCITY_UNITS
        ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS
    )

    # Link threading lib
    BoB_add_link_libraries(${CMAKE_THREAD_LIBS_INIT})

    foreach(target IN LISTS BOB_TARGETS)
        if(NOT "${target}" STREQUAL "")
            target_link_libraries(${target} PUBLIC $ENV{BOB_LINK_LIBS})
        endif()
    endforeach()
endfunction()

set(BOB_ROBOTICS_PATH "${CMAKE_CURRENT_LIST_DIR}/..")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/lib)

# Assume we always need plog
BoB_third_party(plog)

# Don't allow in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.
    Please make a new directory (called a build directory) and run CMake from there.
    You may need to remove CMakeCache.txt." )
endif()

# Default include paths
include_directories(${BOB_ROBOTICS_PATH}
                    ${BOB_ROBOTICS_PATH}/include)

# pkg-config used to get packages on *nix
find_package(PkgConfig REQUIRED)

# Assume we always want threading
find_package(Threads REQUIRED)
