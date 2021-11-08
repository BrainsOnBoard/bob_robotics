cmake_minimum_required(VERSION 3.10)
project(robots)

if(TARGET BoBRobotics::robots)
    return()
endif()

# Allow users to choose the type of robot to use with ROBOT_TYPE env var
# or CMake param
if(NOT ROBOT_TYPE)
    if(NOT "$ENV{ROBOT_TYPE}" STREQUAL "")
        set(ROBOT_TYPE $ENV{ROBOT_TYPE})
    elseif(UNIX AND NOT APPLE) # Default to Norbot on Linux
        set(ROBOT_TYPE Tank::Norbot)
    else()
        set(ROBOT_TYPE Tank::DummyTank)
    endif()
endif()
message(STATUS "Robot type: ${ROBOT_TYPE}")

# Define a ROBOT_TYPE macro to be used as a class name in place of Robots::Norbot etc.
add_definitions(-DROBOT_TYPE=BoBRobotics::Robots::${ROBOT_TYPE})

# We also need to know the namespace name sometimes
string(REGEX REPLACE ::[A-Za-z0-9_]+\$ "" ROBOT_TYPE_NAMESPACE ${ROBOT_TYPE})
add_definitions(-DROBOT_TYPE_NAMESPACE=BoBRobotics::Robots::ROBOT_TYPE_NAMESPACE)

# Define a macro specifying each robot type. Uppercase versions of the class + namespace names
# are used, with :: replaced with _, e.g.: Namespace::RobotClass becomes NAMESPACE_ROBOTCLASS
string(TOUPPER ${ROBOT_TYPE} ROBOT_TYPE_UPPER)
string(REGEX REPLACE :: _ ROBOT_TYPE_UPPER ${ROBOT_TYPE_UPPER})
add_definitions(-DROBOT_TYPE_${ROBOT_TYPE_UPPER})

# Extra modules needed for some robot types. The namespace name tells us
# which module to add (e.g. all Ackermann robots live in robots/ackermann).
string(TOLOWER ${ROBOT_TYPE} ROBOT_TYPE_LOWER)
string(REGEX REPLACE :: / ROBOT_TYPE_LOWER ${ROBOT_TYPE_LOWER})
get_filename_component(ROBOT_TYPE_MODULE ${ROBOT_TYPE_LOWER} DIRECTORY)

set(MODULE_NAME robots/${ROBOT_TYPE_MODULE})
find_package(BoBRobotics REQUIRED ${MODULE_NAME})

escape_module_name(MODULE_LIB ${MODULE_NAME})
add_library(BoBRobotics::robots ALIAS bob_${MODULE_LIB})
