#include "os/windows_include.h"

// BoB robotics includes
#include "os/screen.h"

#ifndef _WIN32
// Standard C++ includes
#include <stdexcept>
#endif

namespace BoBRobotics {
namespace OS {
namespace Screen {
#ifndef _WIN32
// it's necessary to include this here so we don't get name clashes elsewhere
extern "C"
{
#include <X11/Xlib.h>
    using XScreen = Screen;
}
#endif

std::pair<int, int>
getResolution()
{
#ifdef _WIN32
    return std::pair<int, int>{ GetSystemMetrics(SM_CXSCREEN),
                                GetSystemMetrics(SM_CYSCREEN) };
#else
    Display *display = XOpenDisplay(nullptr);
    if (!display) {
        throw std::runtime_error("Could not open display");
    }

    XScreen *screen = DefaultScreenOfDisplay(display);
    XCloseDisplay(display);
    if (!screen) {
        throw std::runtime_error("Could not get display's default screen");
    }

    return std::pair<int, int>{ screen->width, screen->height };
#endif
}
} // Screen
} // OS
} // BoBRobotics
