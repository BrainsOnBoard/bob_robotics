cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES antworld.cc
            SHARED_LIB antworld
            BOB_MODULES antworld
            EXTERNAL_LIBS python numpy)

# On gcc I'm getting this warning for static definitions of structs, but it
# doesn't matter if the remaining fields are filled with zeros, so just suppress
# it. -- AD
if(GNU_TYPE_COMPILER)
    add_compile_flags(-Wno-missing-field-initializers)
endif()
