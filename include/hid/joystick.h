#pragma once

#ifdef _WIN32
#include "joystick_windows.h"
#else
#ifdef __linux__
#include "joystick_linux.h"
#else
#include "joystick_dummy.h"
#endif // __linux__
#endif // _WIN32
