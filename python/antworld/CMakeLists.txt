cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(python_antworld)

add_definitions(-DBOB_SHARED_LIB)
if(UNIX)
    string(APPEND CMAKE_CXX_FLAGS " -fPIC")
endif()

find_package(BoBRobotics QUIET REQUIRED COMPONENTS antworld)

# NB: These are extra modules only available to scikit-build
find_package(PythonExtensions REQUIRED)
find_package(NumPy REQUIRED)  # Not to be confused with our own FindNumpy.cmake!
include_directories(${NumPy_INCLUDE_DIRS})

add_library(antworld MODULE antworld.cc)
python_extension_module(antworld)
target_link_libraries(antworld ${BoBRobotics_LIBRARIES})

install(TARGETS antworld LIBRARY DESTINATION antworld)

if(WIN32)
    install(SCRIPT copy_dlls.cmake)
endif(WIN32)
