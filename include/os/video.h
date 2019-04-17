#pragma once
#ifdef __linux__

// Standard C++ includes
#include <string>
#include <vector>
#include <utility>

namespace BoBRobotics {
namespace OS {
namespace Video {
using CameraDevice = std::pair<int, std::string>;

std::string
getCameraName(int deviceNumber);

std::vector<CameraDevice>
getCameras();

} // Video
} // OS
} // BoBRobotics
#endif // __linux__
