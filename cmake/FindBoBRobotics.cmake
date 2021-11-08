# A helper to reduce boilderplate in modules' CMakeLists.txts. Creates a static
# library with the appropriate name and links it against BoB robotics libraries
# etc.
macro(BoB_module)
    escape_module_name(MODULE_NAME ${PROJECT_NAME})
    set(BOB_MODULE_TARGET bob_${MODULE_NAME})

    # NB: Returns from caller, not from this macro!
    if(TARGET BoBRobotics::${MODULE_NAME})
        return()
    endif()

    add_library(${BOB_MODULE_TARGET} STATIC ${ARGV})
    target_compile_definitions(${BOB_MODULE_TARGET} PUBLIC ${BoBRobotics_DEFINITIONS})
    target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${BoBRobotics_INCLUDE_DIRS})
    target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${BoBRobotics_LIBRARIES})

    # Provide a namespaced alias for this target
    add_library(BoBRobotics::${MODULE_NAME} ALIAS ${BOB_MODULE_TARGET})
endmacro()

# Some module names have slashes in them (e.g. robots/tank), so we escape this
# by converting them to double underscores.
function(escape_module_name OV MODULE)
    string(REPLACE / __ MODULE ${MODULE})
    set(${OV} ${MODULE} PARENT_SCOPE)
endfunction()

function(get_module_path OV MODULE)
    set(${OV} "${BoBRobotics_ROOT_DIR}/src/${MODULE}" PARENT_SCOPE)
endfunction(get_module_path)

function(get_module_libraries LIBS_VARNAME)
    set(LIBRARIES BoBRobotics::base)

    foreach(MODULE ${ARGN})
        escape_module_name(MODULE_LIBNAME ${MODULE})
        get_module_path(MODULE_PATH ${MODULE})
        if(EXISTS "${MODULE_PATH}/CMakeLists.txt")
            set(BoBRobotics_${MODULE_LIBNAME}_FOUND TRUE PARENT_SCOPE)

            add_subdirectory(${MODULE_PATH} "${CMAKE_CURRENT_BINARY_DIR}/BoB/modules/${MODULE}")
            list(APPEND LIBRARIES BoBRobotics::${MODULE_LIBNAME})
        else()
            set(BoBRobotics_${MODULE_LIBNAME}_FOUND FALSE PARENT_SCOPE)
        endif()
    endforeach(MODULE ${ARGN})

    set(${LIBS_VARNAME} ${LIBRARIES} PARENT_SCOPE)
endfunction(get_module_libraries)

include(BoBRobotics)
set(BoBRobotics_ROOT_DIR ${BOB_ROBOTICS_PATH})

################################################################################
# We need to clear BoBRobotics_FIND_COMPONENTS explicitly as it appears not to
# be cleared on subsequent calls to find_package() and get_module_libraries()
# can recurse (modules can depend on other modules).
set(MODULES ${BoBRobotics_FIND_COMPONENTS})
unset(BoBRobotics_FIND_COMPONENTS)

# Get BoB modules and their dependencies
get_module_libraries(_MODULE_LIBRARIES ${MODULES})
list(APPEND BoBRobotics_LIBRARIES ${_MODULE_LIBRARIES})

# Pop this variable. We replace /s with __s (e.g. the presence of robots/ev3
# will be signalled with BoBRobotics_robots__ev3_FOUND).
unset(BoBRobotics_FIND_COMPONENTS)
foreach(MODULE IN LISTS MODULES)
    escape_module_name(MODULE_LIBNAME ${MODULE})
    list(APPEND BoBRobotics_FIND_COMPONENTS ${MODULE_LIBNAME})
endforeach()

# Check that we've found the requested components etc.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BoBRobotics REQUIRED_VARS BoBRobotics_ROOT_DIR HANDLE_COMPONENTS)
