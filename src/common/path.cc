// BoB robotics includes
#include "common/path.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace BoBRobotics {
namespace Path {
filesystem::path
getRepoPath()
{
    return filesystem::path{ BOB_ROBOTICS_PATH };
}

filesystem::path
getResourcesPath()
{
    return getRepoPath() / "resources";
}

filesystem::path
getNewPath(const filesystem::path &rootPath, const std::string &extension)
{
    // Put a timestamp in the filename
    std::stringstream ss;
    const auto timer = time(nullptr);
    const auto currentTime = localtime(&timer);
    ss << std::setfill('0')
       << std::setw(2) << currentTime->tm_mday
       << std::setw(2) << currentTime->tm_mon
       << std::setw(2) << currentTime->tm_year - 100
       << "_"
       << std::setw(2) << currentTime->tm_hour
       << std::setw(2) << currentTime->tm_min
       << std::setw(2) << currentTime->tm_sec
       << "_";
    const auto basename = (rootPath / ss.str()).str();

    // Append a number in case we get name collisions
    ss.str(std::string{}); // clear stringstream
    filesystem::path path;
    for (int i = 1; ; i++) {
        ss << i << extension;
        path = basename + ss.str();
        if (!path.exists()) {
            break;
        }
        ss.str(std::string{}); // clear stringstream
    }
    return path;
}
} // Path
} // BoBRobotics
