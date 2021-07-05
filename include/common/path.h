#pragma once

// Third-party includes
#include "third_party/path.h"

// Standard C includes
#include <ctime>

namespace BoBRobotics {
namespace Path {
//! Get the path of the folder containing the currently running program
filesystem::path
getProgramDirectory();

//! Get the path of the currently running program
filesystem::path
getProgramPath();

/**!
 * \brief Get the path to the BoB robotics repository
 *
 * If the current program was built using a git submodule, the path to that is
 * used, otherwise it falls back on the value of the BOB_ROBOTICS_PATH
 * environment variable. Throws std::runtime_error if the path cannot be found.
 */
filesystem::path
getRepoPath();

//! Gets the path to the BoB robotics resources folder, using getRepoPath()
filesystem::path
getResourcesPath();

//! Generate a novel file/directory path, e.g. for a data file
filesystem::path
getNewPath(const filesystem::path &rootPath = getProgramDirectory(),
           const std::string &extension = "");

//! Generate a novel file/directory path, e.g. for a data file
filesystem::path
getNewPath(const std::tm &currentTime,
           const filesystem::path &rootPath = getProgramDirectory(),
           const std::string &extension = "");

} // Path
} // BoBRobotics
