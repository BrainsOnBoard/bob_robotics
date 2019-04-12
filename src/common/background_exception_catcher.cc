// BoB robotics includes
#include "common/assert.h"
#include "common/background_exception_catcher.h"

// Standard C includes
#include <csignal>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <string>

namespace {
std::string
getSignalName(int sig)
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

void
signalHandler(int sig)
{
    BoBRobotics::BackgroundExceptionCatcher::set(std::make_exception_ptr(std::runtime_error("Caught signal: " + getSignalName(sig))));
}
}

namespace BoBRobotics {
bool BackgroundExceptionCatcher::CatcherExists = false;
std::exception_ptr BackgroundExceptionCatcher::ExceptionPtr;

BackgroundExceptionCatcher::BackgroundExceptionCatcher()
{
    BOB_ASSERT(!CatcherExists); // Only allow one instance
    CatcherExists = true;
}

BackgroundExceptionCatcher::~BackgroundExceptionCatcher()
{
    /*
         * Reset signal handlers to default, in case e.g. we get a hang in a
         * destructor and want to ctrl+c out.
         */
    for (auto &sig : m_Signals) {
        std::signal(sig, SIG_DFL);
    }
}

//! Trap specified signals like exceptions
void
BackgroundExceptionCatcher::trapSignals(const std::unordered_set<int> &signals)
{
    std::copy(signals.begin(), signals.end(), std::back_inserter(m_Signals));
    for (auto &sig : m_Signals) {
        std::signal(sig, &signalHandler);
    }
}

//! Checks if the global exception is set and rethrows it if so
void
BackgroundExceptionCatcher::check() const
{
    if (ExceptionPtr) {
        std::rethrow_exception(ExceptionPtr);
    }
}

//! Sets the global exception
void
BackgroundExceptionCatcher::set(const std::exception_ptr &error)
{
    if (CatcherExists) {
        ExceptionPtr = error;
    } else {
        std::rethrow_exception(error);
    }
}
} // BoBRobotics
