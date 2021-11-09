cmake_minimum_required(VERSION 3.10)
project(python_antworld)

add_definitions(-DBOB_SHARED_LIB)
if(UNIX)
    string(APPEND CMAKE_CXX_FLAGS " -fPIC")
endif()

include(../../cmake/BoBRobotics.cmake)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS antworld)
find_package(Python REQUIRED COMPONENTS Development NumPy)

add_library(antworld SHARED antworld.cc)
target_link_libraries(antworld PUBLIC ${BoBRobotics_LIBRARIES}
                      ${Python_LIBRARIES} Python::NumPy)

set_target_properties(antworld PROPERTIES PREFIX "")
if(WIN32)
    set_target_properties(antworld PROPERTIES SUFFIX ".pyd")
endif()

# TODO: Make CMake put the .so file in the right place

# # On gcc I'm getting this warning for static definitions of structs, but it
# # doesn't matter if the remaining fields are filled with zeros, so just suppress
# # it. -- AD
# if(GNU_TYPE_COMPILER)
#     add_compile_flags(-Wno-missing-field-initializers)
# endif()
