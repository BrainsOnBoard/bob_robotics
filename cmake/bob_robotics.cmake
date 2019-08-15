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
                          "GENN_CPU_ONLY"
                          "EXECUTABLE;GENN_MODEL;GAZEBO_PLUGIN"
                          "SOURCES;BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS;OPTIONS"
                          "${ARGV}")
    BoB_set_options()

    if(NOT PARSED_ARGS_SOURCES AND NOT PARSED_ARGS_GAZEBO_PLUGIN)
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

    # Include local *.h files in project. We don't strictly need to do this, but
    # if we don't then they won't be included in generated Visual Studio
    # projects.
    file(GLOB H_FILES "*.h")

    if(PARSED_ARGS_EXECUTABLE)
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

    if(PARSED_ARGS_GAZEBO_PLUGIN)
        get_filename_component(shortname ${PARSED_ARGS_GAZEBO_PLUGIN} NAME)
        string(REGEX REPLACE "\\.[^.]*$" "" target ${shortname})

        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

        # I'm sometimes getting linker errors when ld is linking against the
        # static libs for BoB modules (because Gazebo plugins, as shared libs,
        # are PIC, but the static libs seem not to be). So let's just compile
        # everything as PIC.
        if(GNU_TYPE_COMPILER)
            add_definitions(-fPIC)
        endif()

        # Gazebo plugins are shared libraries
        add_library(${target} SHARED ${PARSED_ARGS_GAZEBO_PLUGIN})
        list(APPEND BOB_TARGETS ${target})

        # We need to link against Gazebo libs
        BoB_external_libraries(gazebo)
    endif()

    # If this project includes a GeNN model...
    if(PARSED_ARGS_GENN_MODEL)
        get_filename_component(genn_model_name "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
        set(genn_model_dir "${CMAKE_CURRENT_BINARY_DIR}/${genn_model_name}_CODE")
        set(genn_model_src "${CMAKE_CURRENT_SOURCE_DIR}/${PARSED_ARGS_GENN_MODEL}")
        set(genn_model_dest "${genn_model_dir}/runner.cc")

        if(NOT GENN_CPU_ONLY)
            if(DEFINED ENV{CPU_ONLY} AND NOT $ENV{CPU_ONLY} STREQUAL 0)
                set(GENN_CPU_ONLY TRUE)
            else()
                set(GENN_CPU_ONLY ${PARSED_ARGS_GENN_CPU_ONLY})
            endif()
        endif(NOT GENN_CPU_ONLY)
        if(GENN_CPU_ONLY)
            message("Building GeNN model for CPU only")
            add_definitions(-DCPU_ONLY)
            set(CPU_FLAG -c)
        else()
            message("Building GeNN model with CUDA")
        endif()

        # Custom command to generate source code with GeNN
        add_custom_command(PRE_BUILD
                           OUTPUT ${genn_model_dest}
                           DEPENDS ${genn_model_src}
                           COMMAND genn-buildmodel.sh
                                   ${genn_model_src}
                                   ${CPU_FLAG}
                                   -i ${BOB_ROBOTICS_PATH}:${BOB_ROBOTICS_PATH}/include
                           COMMENT "Generating source code with GeNN")

        # Custom command to generate librunner.so
        add_custom_command(PRE_BUILD
                           OUTPUT ${genn_model_dir}/librunner.so
                           DEPENDS ${genn_model_dest}
                           COMMAND make -C "${genn_model_dir}")

        add_custom_target(${PROJECT_NAME}_genn_model ALL DEPENDS ${genn_model_dir}/librunner.so)

        # Our targets depend on librunner.so
        BoB_add_include_directories(/usr/include/genn)
        BoB_add_link_libraries(${genn_model_dir}/librunner.so)
        foreach(target IN LISTS BOB_TARGETS)
            add_dependencies(${target} ${PROJECT_NAME}_genn_model)
        endforeach()

        # So code can access headers in the *_CODE folder
        BoB_add_include_directories(${CMAKE_CURRENT_BINARY_DIR})
    endif()

    # Allow users to choose the type of tank robot to use with TANK_TYPE env var
    # or CMake param (defaults to Norbot)
    if(NOT TANK_TYPE)
        if(NOT "$ENV{TANK_TYPE}" STREQUAL "")
            set(TANK_TYPE $ENV{TANK_TYPE})
        else()
            set(TANK_TYPE Norbot)
        endif()
    endif()
    add_definitions(-DTANK_TYPE=${TANK_TYPE} -DTANK_TYPE_${TANK_TYPE})
    message("Default tank robot type (if used): ${TANK_TYPE}")

    # For EV3 (Lego) robots, we need an extra module
    if(${TANK_TYPE} STREQUAL EV3)
        list(APPEND PARSED_ARGS_BOB_MODULES robots/ev3)
    endif()

    # Do linking etc.
    BoB_build()

    # Copy all DLLs over from vcpkg dir. We don't necessarily need all of them,
    # but it would be a hassle to figure out which ones we need.
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

        link_directories("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
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
                          "SOURCES;BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS;OPTIONS"
                          "${ARGV}")
    if(NOT PARSED_ARGS_SOURCES)
        message(FATAL_ERROR "SOURCES not defined for BoB module")
    endif()
    BoB_set_options()

    # Check we're on a supported platform
    check_platform(${PARSED_ARGS_PLATFORMS})

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
    # CMake defaults to 32-bit builds on Windows
    if(WIN32 AND NOT CMAKE_GENERATOR_PLATFORM)
        message(WARNING "CMAKE_GENERATOR_PLATFORM is set to x86. This is probably not what you want!")
    endif()

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

    # On Unix we use pkg-config to find SDL2 or Eigen, because the CMake
    # packages may not be present
    if(NOT UNIX)
        if(NOT TARGET SDL2::SDL2)
            find_package(SDL2)
        endif()
        if(NOT TARGET Eigen3::Eigen)
            find_package(Eigen3 QUIET)
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
    if (NOT GNU_TYPE_COMPILER AND ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang"))
        set(GNU_TYPE_COMPILER TRUE)

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
        add_compile_flags(-Wno-deprecated-copy)

        # Disable optimisation for debug builds
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

        # If we don't do this, I get linker errors on the BrickPi for the net
        # module
        set(CMAKE_EXE_LINKER_FLAGS "-Wl,--allow-multiple-definition")
    endif()

    # Use C++14. On Ubuntu 16.04, seemingly setting CMAKE_CXX_STANDARD doesn't
    # work, so add the compiler flag manually.
    #
    # Conversely, only setting the compiler flag means that the surveyor example
    # mysteriously gets linker errors on Ubuntu 18.04 and my Arch Linux machine.
    #       - AD
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        add_compile_flags(-std=c++14)
    endif()
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    list(REVERSE ${PROJECT_NAME}_LIBRARIES)

    # Link all targets against the libraries
    foreach(target IN LISTS BOB_TARGETS)
        target_link_libraries(${target} ${${PROJECT_NAME}_LIBRARIES})
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
            if(NOT WIN32 AND NOT NO_I2C)
                # If it's a new version of i2c-tools then we need to link
                # against an additonal library
                execute_process(COMMAND "${CMAKE_CURRENT_LIST_DIR}/is_i2c_tools_new.sh"
                                RESULT_VARIABLE rv)
                if(NOT ${rv} EQUAL 0)
                    BoB_add_link_libraries("i2c")
                endif()
            endif()
        elseif(${lib} STREQUAL opencv)
            BoB_find_package(OpenCV REQUIRED)
        elseif(${lib} STREQUAL eigen3)
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
        elseif(${lib} STREQUAL sfml-graphics)
            # It seems like only newer versions of SFML include a CMake package,
            # so use pkg-config on Unix instead, in case we don't have it
            if(UNIX)
                BoB_add_pkg_config_libraries(sfml-graphics)
            else()
                find_package(SFML REQUIRED graphics)
                BoB_add_link_libraries(sfml-graphics)
            endif()
        elseif(${lib} STREQUAL sdl2)
            # On Unix we use pkg-config to find SDL2, because the CMake package may not
            # be present
            if(UNIX)
                BoB_add_pkg_config_libraries(sdl2)
            elseif(NOT SDL2_FOUND)
                message(FATAL_ERROR "Could not find SDL2")
                BoB_add_link_libraries(SDL2::SDL2)
            endif()

            # Sorry, Norbert ;-). I can try to help you install SFML if it helps!
            #       -- Alex
            BoB_deprecated(SDL2 SFML)
        elseif(${lib} STREQUAL glfw3)
            find_package(glfw3 REQUIRED)
            BoB_add_link_libraries(glfw)
            BoB_external_libraries(opengl)

            # Most of the GLFW code has already been updated, but we still
            # need GLFW temporarily for third_party/imgui
            BoB_deprecated(GLFW SFML)
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
        elseif(${lib} STREQUAL gtest)
            find_package(GTest REQUIRED)
            BoB_add_include_directories(${GTEST_INCLUDE_DIRS})
            BoB_add_link_libraries(${GTEST_LIBRARIES})
        elseif(${lib} STREQUAL gazebo)
            # If Gazebo is added as a dependency multiple times (e.g. from
            # multiple CMakeLists.txt files) then I'm getting an error from the
            # target FreeImage::FreeImage being created multiple times - AD
            if(NOT TARGET FreeImage::FreeImage)
                find_package(gazebo REQUIRED)
                BoB_add_include_directories(${GAZEBO_INCLUDE_DIRS})
                BoB_add_link_libraries(${GAZEBO_LIBRARIES})
                link_directories(${GAZEBO_LIBRARY_DIRS})
                add_compile_flags(${GAZEBO_CXX_FLAGS})
            endif()
        elseif(${lib} STREQUAL spineml_simulation)
            # Find where user has installed GeNN
            exec_or_fail("${BOB_ROBOTICS_PATH}/cmake/find_genn.sh")
            string(STRIP "${SHELL_OUTPUT}" GENN_PATH) # Strip newline
            message("GENN_PATH: ${GENN_PATH}")

            BoB_add_include_directories("${GENN_PATH}/include")
            BoB_add_link_libraries("${GENN_PATH}/lib/libspineml_simulator.a"
                                   "${GENN_PATH}/lib/libspineml_common.a"
                                   dl)
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
                add_definitions(-DWITHOUT_NUMPY)
            else()
                execute_process(COMMAND "python" "${BOB_ROBOTICS_PATH}/cmake/find_numpy.py"
                                RESULT_VARIABLE rv
                                OUTPUT_VARIABLE numpy_include_path)

                # If we have numpy then use it, otherwise matplotlibcpp will work without it
                if(${rv} EQUAL 0)
                    BoB_add_include_directories(${numpy_include_path})
                else()
                    add_definitions(-DWITHOUT_NUMPY)
                endif()
            endif()
        else()
            # Extra actions
            if(${module} STREQUAL ev3dev-lang-cpp)
                # Default to BrickPi3
                if(NOT EV3DEV_PLATFORM)
                    set(EV3DEV_PLATFORM "BRICKPI3" CACHE STRING "Target ev3dev platform (EV3/BRICKPI/BRICKPI3/PISTORMS)")
                endif()
                set_property(CACHE EV3DEV_PLATFORM PROPERTY STRINGS "EV3" "BRICKPI" "BRICKPI3" "PISTORMS")
                add_definitions(-DEV3DEV_PLATFORM_${EV3DEV_PLATFORM})
                message("EV3 platform: ${EV3DEV_PLATFORM}")

                BoB_add_link_libraries(ev3dev)
            elseif(${module} STREQUAL imgui)
                BoB_add_link_libraries(imgui)

                # Extra libs needed
                BoB_external_libraries(glew sfml-graphics)

                # Suppress warning
                add_compile_flags(-Wno-stringop-truncation)
            endif()

            # Checkout git submodules under this path
            find_package(Git REQUIRED)
            exec_or_fail(${GIT_EXECUTABLE} submodule update --init --recursive third_party/${module}
                         WORKING_DIRECTORY ${BOB_ROBOTICS_PATH})

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
set(BOB_ROBOTICS_PATH "${CMAKE_CURRENT_LIST_DIR}/..")

# If this var is defined then this project is being included in another build
if(NOT DEFINED BOB_DIR)
    if(WIN32)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/bin)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BOB_ROBOTICS_PATH}/bin)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BOB_ROBOTICS_PATH}/bin)
    else()
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
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

    # The vcpkg toolchain in theory should do something like this already, but
    # if we don't do this, then cmake can't find any of vcpkg's packages
    file(GLOB children "$ENV{VCPKG_ROOT}/installed/${CMAKE_GENERATOR_PLATFORM}-windows/share/*")
    foreach(child IN LISTS children)
        if(IS_DIRECTORY "${child}")
            list(APPEND CMAKE_PREFIX_PATH "${child}")
        endif()
    endforeach()

    # Suppress warnings about std::getenv being insecure
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
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
