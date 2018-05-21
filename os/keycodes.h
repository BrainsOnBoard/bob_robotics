#pragma once

namespace GeNN_Robotics {
namespace OS {
namespace KeyCodes {
#ifdef _WIN32
enum Key
{
    Left = 0x250000,
    Up = 0x260000,
    Right = 0x270000,
    Down = 0x280000,
    Escape = 0x1b
};
#else
enum Key
{
    Left = 0xff51,
    Up,
    Right,
    Down,
    Escape = 0x1b
};
#endif
} // KeyCodes
} // OS
} // GeNN_Robotics
