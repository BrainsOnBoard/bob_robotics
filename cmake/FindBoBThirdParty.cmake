if(NOT BoBThirdParty_FIND_COMPONENTS)
    message(FATAL_ERROR "Third-party components must be specified")
endif()

if(NOT TARGET BoBRobotics::base)
    include(BoBRobotics)
    make_base_target()
endif()

unset(BoBThirdParty_INCLUDE_DIRS)
set(BoBThirdParty_LIBRARIES BoBRobotics::base)
unset(BoBThirdParty_DEFINITIONS)

foreach(MODULE IN LISTS BoBThirdParty_FIND_COMPONENTS)
    if(TARGET BoBThirdParty::${MODULE})
        continue()
    endif()

    set(FOUND FALSE)
    unset(DEFINITIONS)
    unset(INCLUDE_DIRS)
    unset(LIBRARIES)

    # If the module lives in its own folder, we do some extra things. We skip
    # this step if the module import has already failed.
    set(MODULE_PATH "${BOB_ROBOTICS_PATH}/third_party/${MODULE}")
    if(EXISTS ${MODULE_PATH})
        set(FOUND TRUE)
        set(INCLUDE_DIRS "${MODULE_PATH}" "${MODULE_PATH}/include")

        # Checkout git submodules under this path
        find_package(Git REQUIRED)
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive third_party/${MODULE}
                        WORKING_DIRECTORY "${BOB_ROBOTICS_PATH}"
                        RESULT_VARIABLE RV
                        OUTPUT_VARIABLE SHELL_OUTPUT)
        if(NOT ${RV} EQUAL 0)
            message(WARNING "Checking out git submodule failed")
        endif()

        # If this folder is a cmake project, then build it
        if(EXISTS ${MODULE_PATH}/CMakeLists.txt)
            add_subdirectory(${MODULE_PATH} "${CMAKE_CURRENT_BINARY_DIR}/BoB/third_party/${MODULE}")
        endif()
    endif()

    # Extra actions for third-party modules. These files can flag that the
    # module is not available (e.g. because of missing dependencies) by setting
    # BoBThirdParty_${MODULE}_FOUND to FALSE.
    set(INCPATH "${BOB_ROBOTICS_PATH}/cmake/third_party/${MODULE}.cmake")
    if(EXISTS "${INCPATH}")
        set(FOUND TRUE)
        include("${INCPATH}")
    endif()

    if(FOUND)
        add_library(btp_${MODULE} INTERFACE)
        add_library(BoBThirdParty::${MODULE} ALIAS btp_${MODULE})

        target_compile_definitions(btp_${MODULE} INTERFACE ${DEFINITIONS})
        target_include_directories(btp_${MODULE} INTERFACE
                                   "${BOB_ROBOTICS_PATH}" ${INCLUDE_DIRS})
        target_link_libraries(btp_${MODULE} INTERFACE BoBRobotics::base ${LIBRARIES})

        list(APPEND BoBThirdParty_DEFINITIONS ${DEFINITIONS})
        list(APPEND BoBThirdParty_INCLUDE_DIRS ${INCLUDE_DIRS})
        list(APPEND BoBThirdParty_LIBRARIES BoBThirdParty::${MODULE})
    endif()
endforeach()

foreach(MODULE IN LISTS BoBThirdParty_FIND_COMPONENTS)
    if(TARGET BoBThirdParty::${MODULE})
        set(BoBThirdParty_${MODULE}_FOUND TRUE)
        list(APPEND BoBThirdParty_LIBRARIES BoBThirdParty::${MODULE})
    else()
        set(BoBThirdParty_${MODULE}_FOUND FALSE)
    endif()
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BoBThirdParty REQUIRED_VARS
                                  BoBThirdParty_LIBRARIES HANDLE_COMPONENTS)
