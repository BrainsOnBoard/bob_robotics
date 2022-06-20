cmake_minimum_required(VERSION 3.10)
project(bob_robotics)

add_subdirectory(examples)
add_subdirectory(projects)
add_subdirectory(tests)
add_subdirectory(tests/navigation)
add_subdirectory(tools)

# The included Python code can't actually be built by CMake in this way (you
# need to use the setup.py script), but we include it anyway for IDEs that use
# CMake to get include paths etc. so you can get autocomplete. Hence if we
# *actually* want to test that the code builds (i.e. via Jenkins) then including
# this subdirectory will break things. (The Python build is tested separately.)
if(NOT BUILD_TEST)
    add_subdirectory(python)
endif()
