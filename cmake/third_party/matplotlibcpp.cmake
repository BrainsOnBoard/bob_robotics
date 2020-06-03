BoB_external_libraries(python)

# Try to find numpy path
execute_process(COMMAND "python" "${BOB_ROBOTICS_PATH}/bin/find_numpy.py"
                RESULT_VARIABLE rv
                OUTPUT_VARIABLE numpy_include_path)

# If we have numpy then use it, otherwise matplotlibcpp will still work without it
if(${rv} EQUAL 0)
    BoB_add_include_directories(${numpy_include_path})
else()
    add_definitions(-DWITHOUT_NUMPY)
endif()
