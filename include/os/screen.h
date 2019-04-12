/*
 * Functions for getting screen properties etc. Currently just includes one
 * function which gets the resolution of the main screen.
 */

#pragma once

#ifdef _WIN32
#pragma comment(lib, "user32.lib")
#endif

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace OS {
namespace Screen {
cv::Size
getResolution();
} // Screen
} // OS
} // BoBRobotics
