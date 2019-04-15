cmake_minimum_required(VERSION 3.1)

macro(ADD_CXXFLAGS EXTRA_ARGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(ADD_LDFLAGS EXTRA_ARGS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(BoB_project NAME)
    project(${NAME})
    add_executable(${NAME} ${NAME}.cc)
endmacro()

function(BoB_modules)
    foreach(module IN LISTS ARGV)
        add_subdirectory(${BOB_ROBOTICS_PATH}/src/${module} ${BOB_ROBOTICS_PATH}/src/${module}/build)
        add_dependencies(${PROJECT_NAME} bob_${module})
        set(ENV{BOB_LIBS} "$ENV{BOB_LIBS};${BOB_ROBOTICS_PATH}/lib/libbob_${module}.a")
    endforeach()
endfunction()

set(BOB_ROBOTICS_PATH "${CMAKE_CURRENT_LIST_DIR}/..")
include_directories(${BOB_ROBOTICS_PATH}
                    ${BOB_ROBOTICS_PATH}/include
                    ${BOB_ROBOTICS_PATH}/third_party/plog/include)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    ADD_CXXFLAGS("-std=c++14 -Wall -Wpedantic -Wextra")
    ADD_LDFLAGS("-pthread")
endif()

link_directories(${BOB_ROBOTICS_PATH}/lib)

add_compile_definitions(
    DISABLE_PREDEFINED_UNITS
    ENABLE_PREDEFINED_LENGTH_UNITS
    ENABLE_PREDEFINED_TIME_UNITS
    ENABLE_PREDEFINED_ANGLE_UNITS
    ENABLE_PREDEFINED_VELOCITY_UNITS
    ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS
)
