unset(GeNN_ROOT_DIR)
unset(GeNN_BACKENDS)
unset(GeNN_INCLUDE_DIRS)

macro(find_backend NAME LIBNAME)
    if(EXISTS "${GeNN_ROOT_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}genn_${LIBNAME}_backend${CMAKE_STATIC_LIBRARY_SUFFIX}")
        list(APPEND GeNN_BACKENDS ${NAME})
    endif()
endmacro()

if(WIN32)
    set(_GeNN_BUILDMODEL_NAME genn-buildmodel.bat)
else()
    set(_GeNN_BUILDMODEL_NAME genn-buildmodel.sh)
endif()

# Let users give an explicit path to GeNN
if(GENN_PATH)
    set(GeNN_ROOT_DIR "${GENN_PATH}")
    set(GeNN_BUILDMODEL "${GENN_PATH}/bin/${_GeNN_BUILDMODEL_NAME}")
else()
    # Find genn-buildmodel (which should be in the path)
    find_program(GeNN_BUILDMODEL ${_GeNN_BUILDMODEL_NAME})

    if(GeNN_BUILDMODEL)
        # Figure out path to GeNN
        get_filename_component(_GeNN_BIN_PATH "${GeNN_BUILDMODEL}" DIRECTORY)
        get_filename_component(GeNN_ROOT_DIR "${_GeNN_BIN_PATH}/.." ABSOLUTE)
    endif()
endif()

if(GeNN_ROOT_DIR)
    set(GeNN_INCLUDE_DIRS "${GeNN_ROOT_DIR}/include/genn"
                          "${GeNN_ROOT_DIR}/include/genn/genn")

    # If GeNN is living in its git repo, then this is where the userproject
    # headers will be
    if(EXISTS "${GeNN_ROOT_DIR}/userproject/include")
        list(APPEND GeNN_INCLUDE_DIRS "${GeNN_ROOT_DIR}/userproject/include")
    endif()

    find_backend(CPU single_threaded_cpu)
    find_backend(CUDA cuda)
    find_backend(OPENCL opencl)

    # NB: This currently fails for me with GeNN v4.6.0 because the pugixml
    # library is not bundled
    if(spineml IN_LIST GeNN_FIND_COMPONENTS)
        set(GeNN_SPINEML_INCLUDE_DIRS "${GeNN_ROOT_DIR}/include/spineml")

        # Extra libraries for SpineML integration
        foreach(LIB simulator common)
            list(APPEND GeNN_SPINEML_LIBRARIES "${GeNN_ROOT_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}spineml_${LIB}${CMAKE_STATIC_LIBRARY_SUFFIX}")
        endforeach()

        if(UNIX)
            list(APPEND GeNN_SPINEML_LIBRARIES dl)
        endif()

        list(APPEND GeNN_INCLUDE_DIRS "${GeNN_SPINEML_INCLUDE_DIRS}")
        list(APPEND GeNN_LIBRARIES "${GeNN_SPINEML_LIBRARIES}")
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeNN REQUIRED_VARS GeNN_ROOT_DIR GeNN_BACKENDS)

# cmake_policy(SET CMP0057 NEW)
if(CUDA IN_LIST GeNN_BACKENDS)
    set(GeNN_DEFAULT_BACKEND CUDA)
elseif(OPENCL IN_LIST GeNN_BACKENDS)
    set(GeNN_DEFAULT_BACKEND OPENCL)
else()
    set(GeNN_DEFAULT_BACKEND CPU)
endif()
message(STATUS "GeNN: Available backends: ${GeNN_BACKENDS} (default: ${GeNN_DEFAULT_BACKEND})")

function(add_genn_model TARGET_NAME MODEL_SRC)
    get_filename_component(MODEL_SRC ${MODEL_SRC} ABSOLUTE)

    ############################################################################
    # Parse optional input arguments
    cmake_parse_arguments(PARSED_ARGS
                          ""
                          "BACKEND;CXX_STANDARD;SHARED_LIBRARY_MODEL"
                          "INCLUDE_DIRS" ${ARGN})
    if(PARSED_ARGS_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "Unknown options: ${PARSED_ARGS_UNPARSED_ARGUMENTS}")
    endif()

    # Let users explicitly set this option either globally with the
    # GENN_CPU_ONLY option or by passing CPU_ONLY to this function
    if(PARSED_ARGS_BACKEND)
        string(TOUPPER PARSED_ARGS_BACKEND ${PARSED_ARGS_BACKEND})
        if(NOT "${PARSED_ARGS_BACKEND}" IN_LIST GeNN_BACKENDS)
            message(FATAL_ERROR "Unsupported GeNN backend: ${PARSED_ARGS_BACKEND}")
        endif()
        set(BACKEND ${PARSED_ARGS_BACKEND})
    else()
        set(BACKEND ${GeNN_DEFAULT_BACKEND})
    endif()

    if(${BACKEND} STREQUAL CPU)
        set(BUILDMODEL_OPTIONS -c)
    elseif(${BACKEND} STREQUAL OPENCL)
        set(BUILDMODEL_OPTIONS -o)
    endif()

    # genn_buildmodel accepts a list of extra include dirs separated with colons
    if(PARSED_ARGS_INCLUDE_DIRS)
        string(REPLACE ";" ":" INCLUDE_DIRS "${PARSED_ARGS_INCLUDE_DIRS}")
        list(APPEND BUILDMODEL_OPTIONS "-i${INCLUDE_DIRS}")
    endif()

    ############################################################################
    # Add custom commands for building model with GeNN

    # Explicitly set the C++ standard
    if(PARSED_ARGS_CXX_STANDARD)
        list(APPEND BUILDMODEL_OPTIONS -s c++${PARSED_ARGS_CXX_STANDARD})
    elseif(CMAKE_CXX_STANDARD)
        list(APPEND BUILDMODEL_OPTIONS -s c++${CMAKE_CXX_STANDARD})
    endif()

    if(NOT DEFINED PARSED_ARGS_SHARED_LIBRARY_MODEL OR PARSED_ARGS_SHARED_LIBRARY_MODEL)
        set(SHARED_LIBRARY_MODEL TRUE)
    endif()
    if(SHARED_LIBRARY_MODEL)
        set(MODEL_NAME ${TARGET_NAME})
    else()
        get_filename_component(MODEL_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
    endif()
    get_filename_component(MODEL_DIR "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}_CODE" ABSOLUTE)
    set(MODEL_DST "${MODEL_DIR}/runner.cc")
    set(MODEL_LIB "${MODEL_DIR}/librunner.so")

    # Command to compile model source code
    add_custom_command(OUTPUT "${MODEL_LIB}"
                       DEPENDS "${MODEL_SRC}"
                       IMPLICIT_DEPENDS CXX "${MODEL_SRC}"
                       COMMAND "${GeNN_BUILDMODEL}"
                               ${BUILDMODEL_OPTIONS}
                               "${MODEL_SRC}" > genn.log
                       COMMAND make -C "${MODEL_DIR}"
                       COMMENT "GeNN: Building model (backend: ${BACKEND})"
                       VERBATIM)
    add_custom_target(${TARGET_NAME}_runner DEPENDS "${MODEL_LIB}")

    ############################################################################
    # Add target
    add_library(${TARGET_NAME} INTERFACE)
    add_dependencies(${TARGET_NAME} ${TARGET_NAME}_runner)

    # Export needed include dirs
    target_include_directories(${TARGET_NAME} INTERFACE ${GeNN_INCLUDE_DIRS}
                               ${CMAKE_CURRENT_BINARY_DIR})

    # Projects using SharedLibraryModel dynamically load the model at runtime
    if(NOT DEFINED PARSED_ARGS_SHARED_LIBRARY_MODEL OR PARSED_ARGS_SHARED_LIBRARY_MODEL)
        target_link_libraries(${TARGET_NAME} INTERFACE dl)
    else()
        # Link against librunner.so
        target_link_libraries(${TARGET_NAME} INTERFACE "${MODEL_LIB}")
    endif()

endfunction(add_genn_model)
