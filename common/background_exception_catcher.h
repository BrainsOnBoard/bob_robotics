#pragma once

// BoB robotics includes
#include "assert.h"

// Standard C includes
#include <csignal>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

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
        BOB_ASSERT(!CatcherExists); // Only allow one instance
        CatcherExists = true;
    }

    ~BackgroundExceptionCatcher()
    {
        /*
         * Reset signal handlers to default, in case e.g. we get a hang in a
         * destructor and want to ctrl+c out.
         */
        for (auto &sig : m_Signals) {
            std::signal(sig, SIG_DFL);
        }
    }

#ifdef DEBUG
#define DEFAULT_TRAPPED_SIGNALS { SIGINT }
#else
#define DEFAULT_TRAPPED_SIGNALS { SIGSEGV, SIGINT, SIGFPE }
#endif

    //! Trap specified signals like exceptions
    void trapSignals(const std::unordered_set<int> &signals = DEFAULT_TRAPPED_SIGNALS)
    {
        std::copy(signals.begin(), signals.end(), std::back_inserter(m_Signals));
        for (auto &sig : m_Signals) {
            std::signal(sig, &signalHandler);
        }
    }

    //! Checks if the global exception is set and rethrows it if so
    void check()
    {
        if (ExceptionPtr) {
            std::rethrow_exception(ExceptionPtr);
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

private:
    std::vector<int> m_Signals;
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
