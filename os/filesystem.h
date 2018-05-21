#pragma once

// C++ includes
#include <algorithm>
#include <string>

// OS-specific includes
#ifdef _WIN32
#include "windows_include.h"
#include <shlwapi.h>
#pragma comment(lib, "shlwapi.lib")
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace GeNNRobotics {
namespace OS {
namespace FileSystem {
/*
 * Return true if the specified file path exists, false otherwise.
 */
inline bool fileExists(const std::string &filePath)
{
#ifdef _WIN32
    return PathFileExistsA(filePath.c_str());
#else
    struct stat buffer;
    return (stat(filePath.c_str(), &buffer) == 0);
#endif
}

/*
 * Get file extension.
 */
std::string getExtension(const std::string &filePath)
{
    // find last dot
    size_t dot = filePath.rfind('.');
    if (dot == std::string::npos) {
        // no extension
        return "";
    }

    // get extension
    std::string ext = filePath.substr(dot);

    // make extension lowercase
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    return ext;
}

/*
 * Breaks a file path into path and file name (with extension).
 */
void getFileParts(std::string filePath, std::string &path, std::string &name)
{
    // find last slash
    size_t slash = filePath.rfind('/');
    if (slash == std::string::npos) {
        slash = 0;
    } else {
        slash++;
    }

    path = filePath.substr(0, slash);
    name = filePath.substr(slash);
}
} // FileSystem
} // OS
} // GeNNRobotics
