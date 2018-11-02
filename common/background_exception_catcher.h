#pragma once

// BoB robotics includes
#include "assert.h"

// Standard C includes
#include <csignal>

// Standard C++ includes
#include <array>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::BackgroundExceptionCatcher
//----------------------------------------------------------------------------
//! A wrapper for passing exceptions between threads (i.e. a background thread to the main one)
class BackgroundExceptionCatcher
{
public:
    BackgroundExceptionCatcher()
    {
        // We can have only one instance
        BOB_ASSERT(!CatcherExists);
    }

    ~BackgroundExceptionCatcher()
    {
        // Unregister signal handlers
        for (auto sig : m_Signals) {
            std::signal(sig, SIG_DFL);
        }

        CatcherExists = false;
    }

#ifdef DEBUG
#define DEFAULT_TRAPPED_SIGNALS { SIGINT }
#else
#define DEFAULT_TRAPPED_SIGNALS { SIGSEGV, SIGINT, SIGFPE }
#endif
void trapSignals(const std::unordered_set<int> &signals = DEFAULT_TRAPPED_SIGNALS)
{
}
    // Don't catch segfaults etc. when debugging: we want the debugger to catch them
    void trapSignals(const std::unordered_set<int> &signals = { SIGINT })
#else
    //! Trap specified signals like exceptions
    void trapSignals(const std::unordered_set<int> &signals = { SIGSEGV, SIGINT, SIGFPE })
#endif
    {
        m_Signals.insert(signals.cbegin(), signals.cend());
        for (auto &sig : signals) {
            std::signal(sig, &signalHandler);
        }
    }

    //! Sets the global exception
    static void set(const std::exception_ptr &error)
    {
        if (CatcherExists) {
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
    std::unordered_set<int> m_Signals;
    static bool CatcherExists;
    static std::exception_ptr ExceptionPtr;

    static void signalHandler(int sig)
    {
        set(std::make_exception_ptr(std::runtime_error("Caught signal: " + getSignalName(sig))));
    }

    static std::string getSignalName(int sig)
    {
        switch (sig) {
        case SIGABRT:
            return "SIGABRT";
        case SIGFPE:
            return "SIGFPE";
        case SIGILL:
            return "SIGILL";
        case SIGINT:
            return "SIGINT";
        case SIGSEGV:
            return "SIGSEGV";
        case SIGTERM:
            return "SIGTERM";
        default:
            return "unknown (" + std::to_string(sig) + ")";
        }
    }
}; // BackgroundExceptionCatcher

/*
 * We want to *not* include definitions when we're linking against another
 * object file. This currently only seems to be a problem when linking with
 * libbebop.
 */
#ifndef NO_BACKGROUND_EXCEPTION_CATCHER_DEFINITIONS
bool BackgroundExceptionCatcher::CatcherExists = false;
std::exception_ptr BackgroundExceptionCatcher::ExceptionPtr;
#endif
} // BoBRobotics
