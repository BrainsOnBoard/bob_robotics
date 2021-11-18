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
install(TARGETS _antworld LIBRARY DESTINATION bobrobotics/antworld)
target_link_libraries(_antworld ${BoBRobotics_LIBRARIES})

if(WIN32)
    install(SCRIPT copy_dlls.cmake)
endif(WIN32)

# Some of the python structs have a load of fields that we don't use, so let's
# just auto-initialise them to zeros.
include(CheckCXXCompilerFlag)
foreach(FLAG -Wno-missing-field-initializers)
    check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORTS_${FLAG})
    if(COMPILER_SUPPORTS_${FLAG})
        target_compile_options(_antworld PRIVATE ${FLAG})
    endif()
endforeach()
