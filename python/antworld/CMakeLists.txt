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

add_library(_antworld MODULE antworld.cc)
python_extension_module(_antworld)
install(TARGETS _antworld LIBRARY DESTINATION antworld)
target_link_libraries(_antworld ${BoBRobotics_LIBRARIES})

# set_target_properties(antworld PROPERTIES PREFIX "")
# if(WIN32)
#     set_target_properties(antworld PROPERTIES SUFFIX ".pyd")
# endif()

# # On gcc I'm getting this warning for static definitions of structs, but it
# # doesn't matter if the remaining fields are filled with zeros, so just suppress
# # it. -- AD
# if(GNU_TYPE_COMPILER)
#     add_compile_flags(-Wno-missing-field-initializers)
# endif()
