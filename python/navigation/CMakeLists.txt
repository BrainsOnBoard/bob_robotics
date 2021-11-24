cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(python_navigation)

add_definitions(-DBOB_SHARED_LIB)
if(NOT MSVC)
    string(APPEND CMAKE_CXX_FLAGS " -fPIC")
endif()

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)

# NB: These are extra modules only available to scikit-build
find_package(PythonExtensions REQUIRED)
find_package(NumPy REQUIRED)  # Not to be confused with our own FindNumpy.cmake!
include_directories(${NumPy_INCLUDE_DIRS})

add_library(_navigation MODULE src/navigation.cc)
python_extension_module(_navigation)
install(TARGETS _navigation LIBRARY DESTINATION bob_robotics/navigation)
target_link_libraries(_navigation ${BoBRobotics_LIBRARIES})

if(NOT MSVC)
    target_compile_options(_navigation PRIVATE -Wno-pmf-conversions)
endif()

if(WIN32)
    install(SCRIPT copy_dlls.cmake)
endif(WIN32)
