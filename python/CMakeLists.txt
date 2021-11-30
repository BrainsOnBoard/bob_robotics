# Note that building the Python modules from this folder won't actually produce
# usable Python packages. Just the C++ code will be build. However, this is
# still useful a) for build-testing via Jenkins and b) IDEs which use CMake to
# determine include paths etc. (e.g. VS code).
cmake_minimum_required(VERSION 3.10)
project(python)

find_package(PythonInterp 3.0)
if(NOT PYTHON_EXECUTABLE)
    message(WARNING "Python executable not found")
    return()
endif()

# scikit-build supplies its own CMake modules, so we need to find its path so we
# can include them
execute_process(COMMAND "${PYTHON_EXECUTABLE}" -c
        "from __future__ import print_function\ntry: import os, skbuild; print(os.path.dirname(skbuild.__file__), end='')\nexcept:pass\n"
        OUTPUT_VARIABLE SKBUILD_PATH)
if(NOT SKBUILD_PATH)
    message(WARNING "Scikit-build not found")
    return()
endif()

list(APPEND CMAKE_MODULE_PATH "${SKBUILD_PATH}/resources/cmake")

include(../cmake/build_all.cmake)
