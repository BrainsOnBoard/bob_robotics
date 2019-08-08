#pragma once

// Third-party includes
#include "third_party/path.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
filesystem::path
getNewPath(const filesystem::path &rootPath, const std::string &extension = "");

} // BoBRobotics
