#pragma once

// Standard C++ includes
#include <chrono>

namespace BoBRobotics {
class Stopwatch
{
public:
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Duration = std::chrono::high_resolution_clock::duration;

    //! Start measuring elapsed time from now
    void start();

    //! Check if stopwatch has been started
    bool started() const;

    //! Reset stopwatch to zero
    void reset();

    //! Returns elapsed time since start() was called
    Duration elapsed() const;

    //! Returns the current elapsed time and restarts the Stopwatch
    Duration lap();

    //! Get the current time
    static TimePoint now();

private:
    TimePoint m_StartTime = TimePoint::min();
}; // Stopwatch
} // BoBRobotics
