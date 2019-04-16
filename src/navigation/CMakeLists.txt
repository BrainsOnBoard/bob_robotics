cmake_minimum_required(VERSION 3.1)
include(../../make_common/common.cmake)
BoB_build_module(navigation)
BoB_modules(common imgproc)

find_package(PythonLibs REQUIRED)
add_compile_definitions(WITHOUT_NUMPY)
include_directories(${PYTHON_INCLUDE_DIRS})
