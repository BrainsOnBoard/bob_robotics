cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(python_antworld)

add_definitions(-DBOB_SHARED_LIB)
if(UNIX)
    string(APPEND CMAKE_CXX_FLAGS " -fPIC")
endif()

find_package(BoBRobotics QUIET REQUIRED COMPONENTS antworld)

# NB: These are extra modules only available to scikit-build
find_package(PythonExtensions 3.0 REQUIRED)
find_package(NumPy REQUIRED)  # Not to be confused with our own FindNumpy.cmake!
include_directories(${NumPy_INCLUDE_DIRS})

add_library(_antworld MODULE antworld.cc)
python_extension_module(_antworld)
install(TARGETS _antworld LIBRARY DESTINATION antworld)
target_link_libraries(_antworld ${BoBRobotics_LIBRARIES})

install(TARGETS _antworld LIBRARY DESTINATION antworld)

if(WIN32)
    install(SCRIPT copy_dlls.cmake)
endif(WIN32)
