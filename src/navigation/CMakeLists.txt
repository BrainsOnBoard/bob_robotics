cmake_minimum_required(VERSION 2.8)
include(../../make_common/module.cmake)
BoB_build_module(navigation)

find_package(OpenCV REQUIRED)
target_link_libraries(bob_navigation PUBLIC ${OpenCV_LIBS})

find_package(PythonLibs REQUIRED)
add_compile_definitions(WITHOUT_NUMPY)
include_directories(${PYTHON_INCLUDE_DIRS})
