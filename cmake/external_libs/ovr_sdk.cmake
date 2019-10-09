if(NOT DEFINED ENV{OCULUS_DIR})
    message(FATAL_ERROR "Environment variable OCULUS_DIR is not set")
endif()

# Backslashes are interpreted as escape sequences by CMake
string(REPLACE \\ / OCULUS_DIR $ENV{OCULUS_DIR})

BoB_add_include_directories("${OCULUS_DIR}/LibOVR/Include")

# Note that you have to build these libs manually so they're in the right place and you
# get debug libs
BoB_add_link_libraries("${OCULUS_DIR}/LibOVR/Lib/Windows/x64/${CMAKE_BUILD_TYPE}/LibOVR.lib")
