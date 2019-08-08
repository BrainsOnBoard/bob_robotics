#pragma once

// Standard C includes
#include <csignal>

// Standard C++ includes
#include <stdexcept>
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
    BackgroundExceptionCatcher();
    ~BackgroundExceptionCatcher();

#ifdef DEBUG
#define DEFAULT_TRAPPED_SIGNALS { SIGINT }
#else
#define DEFAULT_TRAPPED_SIGNALS { SIGSEGV, SIGINT, SIGFPE }
#endif

    //! Trap specified signals like exceptions
    void trapSignals(const std::unordered_set<int> &signals = DEFAULT_TRAPPED_SIGNALS);

    //! Checks if the global exception is set and rethrows it if so
    void check() const;

    //! Sets the global exception
    static void set(const std::exception_ptr &error);

private:
    std::vector<int> m_Signals;
    static bool CatcherExists;
    static std::exception_ptr ExceptionPtr;
}; // BackgroundExceptionCatcher
} // BoBRobotics
