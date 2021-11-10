cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(common)

set(SOURCES background_exception_catcher.cc geometry.cc macros.cc main.cc
    map_coordinate.cc path.cc pid.cc semaphore.cc serial_interface.cc
    stopwatch.cc string.cc threadable.cc)

find_package(BoBRobotics REQUIRED)
include(BoBRobotics)
if(TARGET BoBRobotics::common)
    return()
endif()

# Optional dependency on libi2c
# TODO: FIND I2C VERSION
find_package(I2C)
if(I2C_FOUND)
    list(APPEND SOURCES bn055_imu.cc dc_motor_featherwing.cc i2c_interface.cc
         lm9ds1_imu.cc pca_9685.cc)
endif()

add_library(common STATIC ${SOURCES})
add_library(BoBRobotics::common ALIAS common)

find_package(BoBThirdParty REQUIRED COMPONENTS plog)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
target_link_libraries(common PUBLIC BoBRobotics::base
                      BoBThirdParty::plog Eigen3::Eigen Threads::Threads
                      ${I2C_LIBRARIES})

# Default include path
target_include_directories(common PUBLIC "${BOB_ROBOTICS_PATH}/include")

include(BoBGetGitCommits)
get_git_commits()

target_compile_definitions(common PUBLIC
                           -DBOB_ROBOTICS_PATH="${BOB_ROBOTICS_PATH}"

                           # Disable some units types for faster compilation
                           DISABLE_PREDEFINED_UNITS
                           ENABLE_PREDEFINED_LENGTH_UNITS
                           ENABLE_PREDEFINED_TIME_UNITS
                           ENABLE_PREDEFINED_ANGLE_UNITS
                           ENABLE_PREDEFINED_VELOCITY_UNITS
                           ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS
                           ENABLE_PREDEFINED_FREQUENCY_UNITS

                           # Extra info is printed in experiment code
                           $<$<BOOL:${IS_EXPERIMENT}>:BOB_IS_EXPERIMENT>

                           # Info about the state of the git tree
                           -DBOB_ROBOTICS_GIT_COMMIT="${BOB_ROBOTICS_GIT_COMMIT}"
                           -DBOB_PROJECT_GIT_COMMIT="${PROJECT_GIT_COMMIT}"
                           $<$<BOOL:${GIT_TREE_DIRTY}>:BOB_GIT_TREE_DIRTY>
                           $<$<BOOL:${GIT_COMMIT_NOT_IN_MASTER}>:BOB_GIT_COMMIT_NOT_IN_MASTER>
                           )

# Different Jetson devices have different user-facing I2C interfaces so read
# the chip ID and add preprocessor macro, stripping trailing newline
if(EXISTS /sys/module/tegra_fuse/parameters/tegra_chip_id)
    file(READ /sys/module/tegra_fuse/parameters/tegra_chip_id TEGRA_CHIP_ID)
    string(STRIP ${TEGRA_CHIP_ID} TEGRA_CHIP_ID)
    target_compile_definitions(common PUBLIC -DTEGRA_CHIP_ID=${TEGRA_CHIP_ID})
    message(STATUS "Tegra chip id: ${TEGRA_CHIP_ID}")
endif()
