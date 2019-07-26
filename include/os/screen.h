/*
 * Functions for getting screen properties etc. Currently just includes one
 * function which gets the resolution of the main screen.
 */

#pragma once

#ifdef _WIN32
#pragma comment(lib, "user32.lib")
#endif

// Standard C++ includes
#include <utility>

namespace BoBRobotics {
namespace OS {
namespace Screen {
std::pair<int, int>
getResolution();
} // Screen
} // OS
} // BoBRobotics
