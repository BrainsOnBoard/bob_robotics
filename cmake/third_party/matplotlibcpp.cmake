find_package(PythonLibs 3.0)
if(PYTHONLIBS_FOUND)
    list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})

    # TODO: Do we need PYTHON_DEBUG_LIBRARIES on Windows?
    list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

    find_package(NumPy)
    if(PYTHON_NUMPY_FOUND)
        list(APPEND INCLUDE_DIRS "${PYTHON_NUMPY_INCLUDE_DIR}")
    else()
        message(WARNING "NumPy was not found; building without")
        list(APPEND DEFINITIONS -DWITHOUT_NUMPY)
    endif()
else()
    set(FOUND FALSE)
endif()
