# Allow user to choose python binary. PYTHON_VERSION_STRING is set by skbuild,
# so it's a useful fallback. The default is just to use the binary called "python".
if(NOT DEFINED ENV{PYTHON_BIN})
    # Windows doesn't name its python binaries python3.8 etc. so this won't work
    if(NOT WIN32)
        string(SUBSTRING "${PYTHON_VERSION_STRING}" 0 3 PYVER)
    endif()
    set(ENV{PYTHON_BIN} python${PYVER})
endif()

# Try to find numpy path
execute_process(COMMAND "$ENV{PYTHON_BIN}" "${BOB_ROBOTICS_PATH}/bin/find_numpy.py"
                RESULT_VARIABLE rv
                OUTPUT_VARIABLE numpy_include_path)

if(${rv} EQUAL 0)
    BoB_add_include_directories(${numpy_include_path})
else()
    message(FATAL_ERROR "Numpy not found (Is it installed? Is python on your path?)")
endif()
