// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"

#ifdef __linux__
#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace BoBRobotics {
namespace Path {
filesystem::path
getProgramDirectory()
{
    return getProgramPath().parent_path();
}

filesystem::path
getProgramPath()
{
#if defined(__linux__)
    char path[PATH_MAX + 1];
    ssize_t len = readlink("/proc/self/exe", path, PATH_MAX);
    BOB_ASSERT(len >= 0);
    path[len] = '\0';
#elif defined(_WIN32)
    wchar_t path[MAX_PATH];
    DWORD len = GetModuleFileNameW(nullptr, path, MAX_PATH);
    BOB_ASSERT(len > 0);
#elif defined(__APPLE__)
    char path[MAXPATHLEN + 1];
    uint32_t len = sizeof(path);
    BOB_ASSERT(_NSGetExecutablePath(path, &len) == 0);
#else
    #error "Unsupported platform"
#endif

    return filesystem::path{ path };
}

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
getNewPath(const std::tm &currentTime, const filesystem::path &rootPath,
           const std::string &extension)
{
    // Put a timestamp in the filename
    std::stringstream ss;
    ss << std::setfill('0')
       << std::setw(4) << currentTime.tm_year + 1900
       << std::setw(2) << currentTime.tm_mon + 1
       << std::setw(2) << currentTime.tm_mday
       << "_"
       << std::setw(2) << currentTime.tm_hour
       << std::setw(2) << currentTime.tm_min
       << std::setw(2) << currentTime.tm_sec;
    const auto fileNameRoot = (rootPath / ss.str()).str();
    filesystem::path path = fileNameRoot + extension;
    if (!path.exists()) {
        return path;
    }

    // Append a number if there is already a database by this name
    ss.str(std::string{}); // clear stringstream
    for (int i = 2; ; i++) {
        ss << "_" << i << extension;
        path = fileNameRoot + ss.str();
        if (!path.exists()) {
            break;
        }
        ss.str(std::string{}); // clear stringstream
    }
    return path;
}

filesystem::path
getNewPath(const filesystem::path &rootPath, const std::string &extension)
{
    const auto timer = time(nullptr);
    const auto currentTime = localtime(&timer);
    return getNewPath(*currentTime, rootPath, extension);
}
} // Path
} // BoBRobotics
