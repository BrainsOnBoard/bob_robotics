#pragma once

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
//! A wrapper for passing exceptions between threads (i.e. a background thread to the main one)
class GlobalException {
public:
    //! Sets the global exception
    static void set(const std::exception_ptr &error)
    {
        m_Exception = error;
    }

    //! Checks if the global exception is set and rethrows it if so
    static void check()
    {
        if (m_Exception) {
            std::rethrow_exception(m_Exception);
        }
    }

private:
    static std::exception_ptr m_Exception;
}; // GlobalException
} // BoBRobotics