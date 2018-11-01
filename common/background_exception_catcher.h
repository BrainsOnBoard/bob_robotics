#pragma once

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::BackgroundExceptionCatcher
//----------------------------------------------------------------------------
//! A wrapper for passing exceptions between threads (i.e. a background thread to the main one)
class BackgroundExceptionCatcher {
public:
    BackgroundExceptionCatcher()
    {
        NumCatchers++;
    }

    ~BackgroundExceptionCatcher()
    {
        NumCatchers--;
    }

    //! Sets the global exception
    static void set(const std::exception_ptr &error)
    {
        if (NumCatchers > 0) {
            ExceptionPtr = error;
        } else {
            std::rethrow_exception(error);
        }
    }

    //! Checks if the global exception is set and rethrows it if so
    static void check()
    {
        if (ExceptionPtr) {
            std::rethrow_exception(ExceptionPtr);
        }
    }

    // Object is non-copyable
    BackgroundExceptionCatcher(const BackgroundExceptionCatcher &) = delete;
    void operator=(const BackgroundExceptionCatcher &) = delete;
    BackgroundExceptionCatcher(BackgroundExceptionCatcher &&) = default;
    BackgroundExceptionCatcher &operator=(BackgroundExceptionCatcher &&) = default;

private:
    static unsigned NumCatchers;
    static std::exception_ptr ExceptionPtr;
}; // BackgroundExceptionCatcher

/*
 * We want to *not* include definitions when we're linking against another
 * object file. This currently only seems to be a problem when linking with
 * libbebop.
 */
#ifndef NO_BACKGROUND_EXCEPTION_CATCHER_DEFINITIONS
unsigned BackgroundExceptionCatcher::NumCatchers = 0;
std::exception_ptr BackgroundExceptionCatcher::ExceptionPtr;
#endif
} // BoBRobotics
