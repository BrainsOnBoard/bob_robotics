find_package(Matlab REQUIRED)
if(NOT MATLAB_FOUND)
    message(FATAL_ERROR "Matlab not found. Did you set MATLAB_ROOT?")
endif()

BoB_add_include_directories(${MATLAB_INCLUDE_DIR})
BoB_add_link_libraries(${MATLAB_LIBRARIES})
