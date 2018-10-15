#pragma once

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::BackgroundException
//----------------------------------------------------------------------------
//! A wrapper for passing exceptions between threads (i.e. a background thread to the main one)
class BackgroundException {
public:
    //! Start catching exceptions on background threads
    static void enableCatching()
    {
        m_DoCatch = true;
    }

    //! Sets the global exception
    static void set(const std::exception_ptr &error)
    {
        if (m_DoCatch) {
            m_Exception = error;
        } else {
            std::rethrow_exception(error);
        }
    }

    //! Checks if the global exception is set and rethrows it if so
    static void check()
    {
        if (m_Exception) {
            std::rethrow_exception(m_Exception);
        }
    }

private:
    static bool m_DoCatch;
    static std::exception_ptr m_Exception;
}; // BackgroundException

bool BackgroundException::m_DoCatch = false;
std::exception_ptr BackgroundException::m_Exception;
} // BoBRobotics