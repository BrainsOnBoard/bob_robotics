#include "os/windows_include.h"

// BoB robotics includes
#include "os/screen.h"

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

cv::Size
getResolution()
{
#ifdef _WIN32
    return cv::Size(GetSystemMetrics(SM_CXSCREEN),
                    GetSystemMetrics(SM_CYSCREEN));
#else
    Display *display = XOpenDisplay(nullptr);
    if (!display) {
        return cv::Size();
    }
    XScreen *screen = DefaultScreenOfDisplay(display);
    XCloseDisplay(display);
    if (!screen) {
        return cv::Size();
    }

    return cv::Size(screen->width, screen->height);
#endif
}
} // Screen
} // OS
} // BoBRobotics
