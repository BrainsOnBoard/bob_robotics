#pragma once

// Third-party includes
#include "third_party/path.h"

namespace BoBRobotics {
/**!
 * \brief Get the path to the BoB robotics repository
 *
 * If the current program was built using a git submodule, the path to that is
 * used, otherwise it falls back on the value of the BOB_ROBOTICS_PATH
 * environment variable. Throws std::runtime_error if the path cannot be found.
 */
filesystem::path
getBoBRoboticsPath();

//! Gets the path to the BoB robotics resources folder, using getBoBRoboticsPath()
filesystem::path
getResourcesPath();
} // BoBRobotics
