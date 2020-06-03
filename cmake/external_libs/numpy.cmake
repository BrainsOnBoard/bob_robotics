# Try to find numpy path
execute_process(COMMAND "python" "${BOB_ROBOTICS_PATH}/bin/find_numpy.py"
                RESULT_VARIABLE rv
                OUTPUT_VARIABLE numpy_include_path)

if(${rv} EQUAL 0)
    BoB_add_include_directories(${numpy_include_path})
else()
    message(FATAL_ERROR "Numpy not found (Is it installed? Is python on your path?)")
endif()
