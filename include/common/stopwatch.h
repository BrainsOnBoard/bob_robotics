#pragma once

// BoBRobotics includes
#include "assert.h"

// Standard C++ includes
#include <chrono>

namespace BoBRobotics {
class Stopwatch
{
public:
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Duration = std::chrono::high_resolution_clock::duration;

    //! Start measuring elapsed time from now
    void start()
    {
        m_StartTime = now();
    }

    //! Check if stopwatch has been started
    bool started() const
    {
        return m_StartTime != TimePoint::min();
    }

    //! Reset stopwatch to zero
    void reset()
    {
        m_StartTime = TimePoint::min();
    }

    //! Returns elapsed time since start() was called
    Duration elapsed() const
    {
        BOB_ASSERT(started());
        return now() - m_StartTime;
    }

    //! Returns the current elapsed time and restarts the Stopwatch
    Duration lap()
    {
        const TimePoint currentTime = now();
        const Duration elapsed = currentTime - m_StartTime;
        m_StartTime = currentTime;
        return elapsed;
    }

    //! Get the current time
    static TimePoint now()
    {
        return std::chrono::high_resolution_clock::now();
    }

private:
    TimePoint m_StartTime = TimePoint::min();
}; // Stopwatch
} // BoBRobotics
