cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(python_navigation)

add_definitions(-DBOB_SHARED_LIB)
if(NOT MSVC)
    string(APPEND CMAKE_CXX_FLAGS " -fPIC")
endif()

find_package(BoBRobotics QUIET REQUIRED COMPONENTS navigation)
find_package(BoBThirdParty REQUIRED COMPONENTS pybind11)

# NB: These are extra modules only available to scikit-build
find_package(PythonExtensions REQUIRED)
find_package(NumPy REQUIRED)  # Not to be confused with our own FindNumpy.cmake!
include_directories(${NumPy_INCLUDE_DIRS})

pybind11_add_module(_navigation MODULE src/navigation.cc)
install(TARGETS _navigation LIBRARY DESTINATION bob_robotics/navigation)
target_link_libraries(_navigation PRIVATE ${BoBRobotics_LIBRARIES})

# Suppress clang warning (emitted by clang-tidy)
set(FLAG -Wno-ignored-optimization-argument)
check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORTS_${FLAG})
if(COMPILER_SUPPORTS_${FLAG})
    target_compile_options(_navigation PRIVATE ${FLAG})
endif()

if(WIN32)
    install(SCRIPT copy_dlls.cmake)
endif(WIN32)
