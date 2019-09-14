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
