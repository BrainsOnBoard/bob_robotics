cmake_minimum_required(VERSION 3.1)

# Build a module from sources in current folder. All *.cc files are compiled
# into a static library.
macro(BoB_module)
    BoB_module_custom(${ARGN})
    BoB_build()
endmacro()

# Build a "project" in the current folder (e.g. example, etc.). Each *.cc file
# found is compiled into a separate executable.
macro(BoB_project)
    BoB_init()

    # Parse input args
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSED_ARGS
                          "INCLUDE_GENN_USERPROJECTS"
                          "EXECUTABLE;PYTHON_MODULE;CXX_STANDARD"
                          "SOURCES;BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS;OPTIONS"
                          "${ARGV}")
    BoB_set_options()

    if(NOT PARSED_ARGS_SOURCES)
        message(FATAL_ERROR "SOURCES not defined for BoB project")
    endif()

    # Check we're on a supported platform
    check_platform(${PARSED_ARGS_PLATFORMS})

    if(PARSED_ARGS_EXECUTABLE)
        set(NAME ${PARSED_ARGS_EXECUTABLE})
    else()
        # Use current folder as project name
        get_filename_component(NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
    endif()
    project(${NAME})

    # Allow for setting the C++ standard on a per-project basis.
    if(PARSED_ARGS_CXX_STANDARD AND NOT DEFINED CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD ${PARSED_ARGS_CXX_STANDARD})
    endif()

    # Include local *.h files in project. We don't strictly need to do this, but
    # if we don't then they won't be included in generated Visual Studio
    # projects.
    file(GLOB H_FILES "*.h")

    if(PARSED_ARGS_PYTHON_MODULE)
        set(NAME ${PARSED_ARGS_PYTHON_MODULE})
        if(WIN32 AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
            set(NAME ${NAME}_d)
        endif()
        add_library(${NAME} SHARED "${PARSED_ARGS_SOURCES}" "${H_FILES}")
        set_target_properties(${NAME} PROPERTIES PREFIX "")
        if(WIN32)
            set_target_properties(${NAME} PROPERTIES SUFFIX ".pyd")
        endif()
        set(BOB_TARGETS ${NAME})
        add_definitions(-DBOB_SHARED_LIB)
        install(TARGETS ${NAME} LIBRARY DESTINATION antworld)

        if(GNU_TYPE_COMPILER)
            add_compile_flags(-fPIC)
        endif()
        BoB_external_libraries(python)
    elseif(PARSED_ARGS_EXECUTABLE)
        # Build a single executable from these source files
        add_executable(${NAME} "${PARSED_ARGS_SOURCES}" "${H_FILES}")
        set(BOB_TARGETS ${NAME})
    else()
        # Build each *.cc file as a separate executable
        foreach(file IN LISTS PARSED_ARGS_SOURCES)
            get_filename_component(shortname ${file} NAME)
            string(REGEX REPLACE "\\.[^.]*$" "" target ${shortname})
            add_executable(${target} "${file}" "${H_FILES}")
            list(APPEND BOB_TARGETS ${target})
        endforeach()
    endif()

    # If this project includes a GeNN model...
    if(PARSED_ARGS_INCLUDE_GENN_USERPROJECTS)
        # Find genn-buildmodel (which should be in the path)
        if(WIN32)
            find_program(GENN_BUILDMODEL genn-buildmodel.bat)
        else()
            find_program(GENN_BUILDMODEL genn-buildmodel.sh)
        endif()
        if(GENN_BUILDMODEL)
            # Remove filename to get path to GeNN bin directory
            get_filename_component(GENN_BIN_PATH ${GENN_BUILDMODEL} DIRECTORY)

            # Get absolute path to userproject include
            get_filename_component(GENN_USERPROJECT ${GENN_BIN_PATH}/../userproject/include ABSOLUTE)
            message("GeNN found in ${GENN_USERPROJECT}")
            BoB_add_include_directories(${GENN_USERPROJECT})

            # On *nix link dl
            if(NOT WIN32)
                BoB_add_link_libraries(dl)
            endif()
        else()
            message(FATAL_ERROR "GeNN not found. Please install and ensure it is in path.")
        endif()
    endif()

    # Allow users to choose the type of tank robot to use with ROBOT_TYPE env var
    # or CMake param
    if(NOT ROBOT_TYPE)
        if(NOT "$ENV{ROBOT_TYPE}" STREQUAL "")
            set(ROBOT_TYPE $ENV{ROBOT_TYPE})
        elseif(UNIX AND NOT APPLE) # Default to Norbot on Linux
            set(ROBOT_TYPE Norbot)
        else()
            set(ROBOT_TYPE Tank)
        endif()
    endif()
    message("Default robot type (if used): ${ROBOT_TYPE}")

    # Define a ROBOT_TYPE macro to be used as a class name in place of Robots::Norbot etc.
    add_definitions(-DROBOT_TYPE=${ROBOT_TYPE})

    # Define a macro specifying each robot type. Uppercase versions of the class + namespace names
    # are used, with :: replaced with _, e.g.: Namespace::RobotClass becomes NAMESPACE_ROBOTCLASS
    string(TOUPPER ${ROBOT_TYPE} ROBOT_TYPE_UPPER)
    string(REGEX REPLACE :: _ ROBOT_TYPE_UPPER ${ROBOT_TYPE_UPPER})
    add_definitions(-DROBOT_TYPE_${ROBOT_TYPE_UPPER})

    # Extra modules needed for some robot types
    if(${ROBOT_TYPE} STREQUAL EV3)
        list(APPEND PARSED_ARGS_BOB_MODULES robots/ev3)
    elseif(${ROBOT_TYPE} STREQUAL Gazebo::Tank)
        list(APPEND PARSED_ARGS_BOB_MODULES robots/gazebo)
    endif()

    # We always need the common module so that main() is defined
    list(APPEND PARSED_ARGS_BOB_MODULES common)

    # Do linking etc.
    BoB_build()

    # When using the ARSDK (Parrot Bebop SDK) the rpath is not correctly set on
    # Ubuntu (and presumably when linking against other libs in non-standard
    # locations too). This linker flag fixes the problem.
    if(UNIX AND NOT APPLE)
        set(CMAKE_EXE_LINKER_FLAGS -Wl,--disable-new-dtags)
    endif()

    # Copy all DLLs over from vcpkg dir. We don't necessarily need all of them,
    # but it would be a hassle to figure out which ones we need.
    if(WIN32)
        # Add custom command to 
        foreach(target IN LISTS BOB_TARGETS)
            add_custom_command(TARGET ${target} POST_BUILD
                COMMAND ${BOB_ROBOTICS_PATH}/bin/copy_dependencies_vcpkg.bat "${CMAKE_SOURCE_DIR}/${target}.exe" "${VCPKG_PACKAGE_DIR}"
            )
        endforeach()
    endif()
endmacro()

macro(BoB_set_options)
    # Extra compile-type options
    if(PARSED_ARGS_OPTIONS)
        foreach(option IN LISTS PARSED_ARGS_OPTIONS)
            if(${option})
                message("Option: ${option}=on")
                set(OPTION_${option} TRUE)
                add_definitions(-D${option})
            else()
                message("Option: ${option}=off")
            endif()
        endforeach()
    endif()
endmacro()

# Build a module with extra libraries etc. Currently used by robots/bebop
# module because the stock BoB_module() isn't flexible enough.
macro(BoB_module_custom)
    BoB_init()

    include(CMakeParseArguments)
    cmake_parse_arguments(PARSED_ARGS
                          ""
                          ""
                          "SOURCES;GAZEBO_PLUGINS;BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS;OPTIONS"
                          "${ARGV}")
    if(NOT PARSED_ARGS_SOURCES)
        message(FATAL_ERROR "SOURCES not defined for BoB module")
    endif()
    BoB_set_options()

    # Check we're on a supported platform
    check_platform(${PARSED_ARGS_PLATFORMS})

    if(PARSED_ARGS_GAZEBO_PLUGINS)
        # I'm sometimes getting linker errors when ld is linking against the
        # static libs for BoB modules (because Gazebo plugins, as shared libs,
        # are PIC, but the static libs seem not to be). So let's just compile
        # everything as PIC.
        if(GNU_TYPE_COMPILER)
            add_definitions(-fPIC)
        endif()

        # We need to link against Gazebo libs
        BoB_external_libraries(gazebo)

        # Dump plugins into source dir
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

        foreach(plugin IN LISTS PARSED_ARGS_GAZEBO_PLUGINS)
            # Use the plugin's filename, minus the extension as target name
            get_filename_component(shortname ${plugin} NAME)
            string(REGEX REPLACE "\\.[^.]*$" "" target ${shortname})

            # Gazebo plugins are shared libraries
            add_library(${target} SHARED ${plugin})
            list(APPEND BOB_TARGETS ${target})
        endforeach()
    endif()

    # Module name is based on path relative to src/
    file(RELATIVE_PATH NAME "${BOB_ROBOTICS_PATH}/src" "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BOB_TARGETS bob_${NAME})
    string(REPLACE / _ BOB_TARGETS ${BOB_TARGETS})
    project(${BOB_TARGETS})

    file(GLOB H_FILES "${BOB_ROBOTICS_PATH}/include/${NAME}/*.h")
    add_library(${BOB_TARGETS} STATIC ${PARSED_ARGS_SOURCES} ${H_FILES})
    set_target_properties(${BOB_TARGETS} PROPERTIES PREFIX ./lib)
    add_definitions(-DNO_HEADER_DEFINITIONS)
endmacro()

macro(BoB_init)
    # For release builds, CMake disables assertions, but a) this isn't what we
    # want and b) it will break code.
    if(MSVC)
        string(REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    else()
        string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    endif()
endmacro()

macro(add_compile_flags EXTRA_ARGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(add_linker_flags EXTRA_ARGS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(always_included_packages)
    # Assume we always want threading
    find_package(Threads REQUIRED)

    # Annoyingly, these packages export a target rather than simply variables
    # with the include path and link flags and it seems that this target isn't
    # "passed up" by add_subdirectory(), so we always include these packages on
    # the off-chance we need them.
    if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND NOT TARGET OpenMP::OpenMP_CXX)
        find_package(OpenMP QUIET)
    endif()
    if(NOT TARGET GLEW::GLEW)
        find_package(GLEW QUIET)
    endif()

    # Gazebo creates this target unconditionally, so make sure we don't try to
    # create it twice
    if(NOT TARGET FreeImage::FreeImage)
        find_package(gazebo QUIET)
    endif()

    # On Unix we use pkg-config to find SDL2 or Eigen, because the CMake
    # packages may not be present
    if(NOT UNIX)
        if(NOT TARGET SDL2::SDL2)
            find_package(SDL2 QUIET)
        endif()
        if(NOT TARGET Eigen3::Eigen)
            find_package(Eigen3 QUIET)
        endif()
    endif()
endmacro()

macro(get_git_commit DIR VARNAME)
    find_package(Git REQUIRED)
    execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DIR}" rev-parse --short HEAD
                    RESULT_VARIABLE rv
                    OUTPUT_VARIABLE ${VARNAME}
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT ${rv} EQUAL 0)
        # Git might fail (e.g. if we're not in a git repo), but let's carry on regardless
        set(${VARNAME} "(unknown)")
    else()
        # Append -dirty if worktree has been modified
        execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DIR}" diff --no-ext-diff --quiet --exit-code
        RESULT_VARIABLE rv)
        if(NOT ${rv} EQUAL 0)
            set(${VARNAME} ${${VARNAME}}-dirty)
        endif()
    endif()
endmacro()

macro(BoB_build)
    # Don't build i2c code if NO_I2C environment variable is set
    if(NOT I2C_MESSAGE_DISPLAYED AND (NO_I2C OR (NOT "$ENV{NO_I2C}" STREQUAL 0 AND NOT "$ENV{NO_I2C}" STREQUAL "")))
        set(I2C_MESSAGE_DISPLAYED TRUE)
        message("NO_I2C is set: not building i2c code")
        set(NO_I2C TRUE)
        add_definitions(-DNO_I2C)
    endif()

    # Add macro so that programs know where the root folder is for e.g. loading
    # resources
    add_definitions(-DBOB_ROBOTICS_PATH="${BOB_ROBOTICS_PATH}")

    # Pass the current git commits of project and BoB robotics as C macros
    get_git_commit("${BOB_ROBOTICS_PATH}" BOB_ROBOTICS_GIT_COMMIT)
    get_git_commit("${CMAKE_SOURCE_DIR}" PROJECT_GIT_COMMIT)
    add_definitions(-DBOB_ROBOTICS_GIT_COMMIT="${BOB_ROBOTICS_GIT_COMMIT}"
                    -DBOB_PROJECT_GIT_COMMIT="${PROJECT_GIT_COMMIT}")

    # Default to building release type
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    endif()
    message("Build type: ${CMAKE_BUILD_TYPE}")

    if(NOT WIN32)
        # Use ccache if present to speed up repeat builds
        find_program(CCACHE_FOUND ccache)
        if(CCACHE_FOUND)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        else()
            message(WARNING "ccache not found. Install for faster repeat builds.")
        endif()
    endif()

    # Set DEBUG macro when compiling in debug mode
    if(${CMAKE_BUILD_TYPE} STREQUAL Debug)
        add_definitions(-DDEBUG)
    endif()

    # Flags for gcc and clang
    if(GNU_TYPE_COMPILER)
        # Default to building with -march=native
        if(NOT DEFINED ENV{ARCH})
            set(ENV{ARCH} native)
        endif()

        # Enable warnings and set architecture
        add_compile_flags("-Wall -Wpedantic -Wextra -march=$ENV{ARCH}")

        # Gcc has an annoying feature where you can mark functions with
        # __attribute__((warn_unused_result)) and then the calling code *has*
        # to do something with the result and can't ignore it; hacks such as
        # (void) annoyingFunction() don't work either. We're mostly
        # seeing this warning for calls to std::system() (in our code and third-
        # party code), but in those cases we generally really don't care about
        # the return value. So let's just disable it globally to save faffing
        # around.
        add_compile_flags(-Wno-unused-result)

        # I'm getting warnings based for code in the Eigen headers, so let's
        # just disable it. I tried setting this flag only when we're actually
        # using Eigen, but that didn't seem to work, and it seems pretty
        # harmless, so it's probably fine to just disable it globally.
        #          - AD
        #
        # Eigen version: 3.3.7
        # gcc version:   9.1.0
        if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.0)
            add_compile_flags(-Wno-deprecated-copy)
        endif()

        # Disable optimisation for debug builds
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    endif()

    # If C++ standard has not been specified explicitly either with a command
    # line argument or with an environment variable, set the standard to C++14,
    # the minimum supported by BoB robotics.
    #
    # The main reason for allowing users to choose a more recent standard is
    # because the latest version of Gazebo (v11) requires C++17, so we need it
    # for Gazebo-based projects.
    if(NOT DEFINED CMAKE_CXX_STANDARD)
        if(DEFINED ENV{BOB_ROBOTICS_CXX_STANDARD})
            set(CMAKE_CXX_STANDARD $ENV{BOB_ROBOTICS_CXX_STANDARD})
        else()
            set(CMAKE_CXX_STANDARD 14)
        endif()
    endif()
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # On Ubuntu 16.04, seemingly setting CMAKE_CXX_STANDARD by itself doesn't
    # work, so add the compiler flag manually.
    #
    # Conversely, only setting the compiler flag means that the surveyor example
    # mysteriously gets linker errors on Ubuntu 18.04 and my Arch Linux machine.
    #       - AD
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        add_compile_flags(-std=gnu++${CMAKE_CXX_STANDARD})
    endif()

    # Irritatingly, neither GCC nor Clang produce nice ANSI-coloured output if they detect
    # that output "isn't a terminal" - which seems to include whatever pipe-magick cmake includes.
    # https://medium.com/@alasher/colored-c-compiler-output-with-ninja-clang-gcc-10bfe7f2b949
    #
    # When compiling via Jenkins though, we don't want colourised output because
    # a) the Jenkins logs don't support colours anyway and b) the colour escape
    # sequences seem to break Jenkins' auto-parsing of error messages.
    if(NOT "$ENV{USER}" STREQUAL jenkins)
        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
            add_compile_flags(-fdiagnostics-color=always)
        elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
            add_compile_flags(-fcolor-diagnostics)
        endif ()
    endif()

    # Different Jetson devices have different user-facing I2C interfaces
    # so read the chip ID and add preprocessor macro
    if(EXISTS /sys/module/tegra_fuse/parameters/tegra_chip_id)
        file(READ /sys/module/tegra_fuse/parameters/tegra_chip_id TEGRA_CHIP_ID)
        add_definitions(-DTEGRA_CHIP_ID=${TEGRA_CHIP_ID})
        message("Tegra chip id: ${TEGRA_CHIP_ID}")
    endif()

    # Set include dirs and link libraries for this module/project
    always_included_packages()
    BoB_external_libraries(${PARSED_ARGS_EXTERNAL_LIBS})
    BoB_third_party(${PARSED_ARGS_THIRD_PARTY})
    BoB_modules(${PARSED_ARGS_BOB_MODULES})

    # Link threading lib
    BoB_add_link_libraries(${CMAKE_THREAD_LIBS_INIT})

    # Clang needs to be linked against libm and libstdc++ explicitly
    if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        BoB_add_link_libraries(m stdc++)
    endif()

    # The list of linked libraries can end up very long with lots of duplicate
    # entries and this can break ld, so remove them. We remove from the start,
    # so that dependencies will always (I think!) be in the right order.
    list(REVERSE ${PROJECT_NAME}_LIBRARIES)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_LIBRARIES)

    # Link all targets against the libraries
    foreach(target IN LISTS BOB_TARGETS)
        target_link_libraries(${target} ${${PROJECT_NAME}_LIBRARIES})

        # Older versions of CMake don't have target_link_directories(), but
        # hopefully we can get away without using it in this case (because we'll
        # probably only find this on Linux machines which don't care about
        # linking directories anyway)
        if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13.0")
            target_link_directories(${target} PUBLIC ${${PROJECT_NAME}_LIB_DIRS})
        endif()
    endforeach()
endmacro()

function(check_platform)
    # If it's an empty list, return
    if(${ARGC} EQUAL 0)
        return()
    endif()

    # Check the platforms are valid
    foreach(platform IN LISTS ARGV)
        if(NOT "${platform}" STREQUAL unix
           AND NOT "${platform}" STREQUAL linux
           AND NOT "${platform}" STREQUAL windows
           AND NOT "${platform}" STREQUAL all)
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
    set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${ARGV}"
        CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)
endmacro()

macro(BoB_add_link_directories)
    if(${CMAKE_VERSION} VERSION_LESS "3.13.0")
        link_directories(${ARGV})
    else()
        set(${PROJECT_NAME}_LIB_DIRS "${${PROJECT_NAME}_LIB_DIRS};${ARGV}"
            CACHE INTERNAL "${PROJECT_NAME}: Library directories" FORCE)
    endif()
endmacro()

function(BoB_deprecated WHAT ALTERNATIVE)
    message(WARNING "!!!!! WARNING: Use of ${WHAT} in BoB robotics code is deprecated and will be removed in future. Use ${ALTERNATIVE} instead. !!!!!")
endfunction()

function(BoB_add_include_directories)
    # Sometimes we get newline characters in an *_INCLUDE_DIRS variable (e.g.
    # with the OpenCV package) and this breaks CMake
    string(REPLACE "\n" " " ARGV "${ARGV}")

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

        # Some (sub)modules have a slash in the name; replace with underscore
        string(REPLACE / _ module_name ${module})

        # All of our targets depend on this module
        foreach(target IN LISTS BOB_TARGETS)
            add_dependencies(${target} bob_${module_name})
        endforeach()

        # Build subdirectory
        if(NOT TARGET bob_${module_name})
            add_subdirectory(${module_path} "${BOB_DIR}/modules/${module_name}")
        endif()

        # Link against BoB module static lib + its dependencies
        BoB_add_link_libraries(bob_${module_name} ${bob_${module_name}_LIBRARIES})
        BoB_add_include_directories(${bob_${module_name}_INCLUDE_DIRS})
    endforeach()
endfunction()

function(BoB_find_package)
    find_package(${ARGV})
    BoB_add_include_directories(${${ARGV0}_INCLUDE_DIRS})
    BoB_add_link_libraries(${${ARGV0}_LIBS} ${${ARGV0}_LIBRARIES})
    BoB_add_link_directories(${${ARGV0}_LIB_DIR})
endfunction()

function(BoB_add_pkg_config_libraries)
    find_package(PkgConfig)
    foreach(lib IN LISTS ARGV)
        pkg_check_modules(${lib} REQUIRED ${lib})
        BoB_add_include_directories(${${lib}_INCLUDE_DIRS})
        BoB_add_link_directories(${${lib}_LIBRARY_DIRS})
        BoB_add_link_libraries(${${lib}_LIBRARIES})
        add_compile_flags(${${lib}_CFLAGS} ${${lib}_LDFLAGS})
    endforeach()
endfunction()

function(BoB_external_libraries)
    foreach(lib IN LISTS ARGV)
        set(incpath "${BOB_ROBOTICS_PATH}/cmake/external_libs/${lib}.cmake")
        if(EXISTS "${incpath}")
            include("${incpath}")
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
        # Extra actions for third-party modules
        set(incpath "${BOB_ROBOTICS_PATH}/cmake/third_party/${module}.cmake")
        if(EXISTS "${incpath}")
            include("${incpath}")
        endif()

        if(EXISTS "${BOB_ROBOTICS_PATH}/third_party/${module}")
            # Checkout git submodules under this path
            find_package(Git REQUIRED)
            exec_or_fail(${GIT_EXECUTABLE} submodule update --init --recursive third_party/${module}
                            WORKING_DIRECTORY "${BOB_ROBOTICS_PATH}")

            # If this folder is a cmake project, then build it
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            if(EXISTS ${module_path}/CMakeLists.txt)
                add_subdirectory(${module_path} "${BOB_DIR}/third_party/${module}")
            endif()

            # Add to include path
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            include_directories(${module_path} ${module_path}/include ${${module}_INCLUDE_DIRS})

            # Link against extra libs, if needed
            BoB_add_link_libraries(${${module}_LIBRARIES})
        endif()
    endforeach()
endfunction()

# Don't allow in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.
    Please make a new directory (called a build directory) and run CMake from there.
    You may need to remove CMakeCache.txt." )
endif()

# Set output directories for libs and executables
get_filename_component(BOB_ROBOTICS_PATH .. ABSOLUTE BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")

# If this var is defined then this project is being included in another build
if(NOT DEFINED BOB_DIR)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
    if(WIN32) 
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR})
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR})
    endif()

    # Folder to build BoB modules + third-party modules
    set(BOB_DIR "${CMAKE_CURRENT_BINARY_DIR}/BoB")
endif()

# Use vcpkg on Windows
if(WIN32)
    # Use vcpkg's cmake toolchain
    if(DEFINED ENV{VCPKG_ROOT})
        if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
            set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
                CACHE STRING "")
        endif()
    else()
        message(FATAL_ERROR "The environment VCPKG_ROOT must be set on Windows")
    endif()

    # When using Visual Studio as a target, you have to specify whether you want
    # a 32- or 64-bit build when CMake is invoked. Use this to determine the
    # correct vcpkg libraries to look for or default to 64-bit.
    if(CMAKE_GENERATOR_PLATFORM)
        set(PLATFORM ${CMAKE_GENERATOR_PLATFORM}-windows)
    else()
        set(PLATFORM x64-windows)
    endif()
    set(VCPKG_PACKAGE_DIR "$ENV{VCPKG_ROOT}/installed/${PLATFORM}")


    # The vcpkg toolchain in theory should do something like this already, but
    # if we don't do this, then cmake can't find any of vcpkg's packages
    file(GLOB children "${VCPKG_PACKAGE_DIR}/share/*")
    foreach(child IN LISTS children)
        if(IS_DIRECTORY "${child}")
            list(APPEND CMAKE_PREFIX_PATH "${child}")
        endif()
    endforeach()

    # Link against vcpkg packages' libs
    link_directories("${VCPKG_PACKAGE_DIR}/lib")

    # Suppress warnings about std::getenv being insecure
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)

    # We don't want the min/max macros defined in windows.h
    add_definitions(-DNOMINMAX)

    # the version of the Windows API that we want
    add_definitions(-D_WIN32_WINNT=_WIN32_WINNT_WIN7)

    # for a less bloated version of windows.h
    add_definitions(-D_WIN32_LEAN_AND_MEAN)

    # disable the winsock v1 API, which is included by default and conflicts
    # with v2 of the API
    add_definitions(-D_WINSOCKAPI_)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(GNU_TYPE_COMPILER TRUE)
endif()

# Assume we always need plog
BoB_third_party(plog)

# Default include paths
include_directories(${BOB_ROBOTICS_PATH}
                    ${BOB_ROBOTICS_PATH}/include)

# Disable some of the units types in units.h for faster compilation
add_definitions(
    -DDISABLE_PREDEFINED_UNITS
    -DENABLE_PREDEFINED_LENGTH_UNITS
    -DENABLE_PREDEFINED_TIME_UNITS
    -DENABLE_PREDEFINED_ANGLE_UNITS
    -DENABLE_PREDEFINED_VELOCITY_UNITS
    -DENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS
)

# Look for additional CMake packages in the current folder
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
