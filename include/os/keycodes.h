#pragma once

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace OS {

#ifdef _WIN32
static const int KeyMask = std::numeric_limits<int>::max();

namespace KeyCodes {
enum Key
{
    Left = 0x250000,
    Up = 0x260000,
    Right = 0x270000,
    Down = 0x280000,
    Escape = 0x1b
};
} // KeyCodes
#else
static const int KeyMask = 0xffff;

namespace KeyCodes {
enum Key
{
    Left = 0xff51,
    Up,
    Right,
    Down,
    Escape = 0x1b
};
} // KeyCodes
#endif

} // OS
} // BoBRobotics
