#pragma once

// Standard C++ includes
#include <chrono>

namespace BoBRobotics {
class Stopwatch
{
public:
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Duration = std::chrono::high_resolution_clock::duration;

    void start()
    {
        m_StartTime = now();
    }

    Duration elapsed() const
    {
        return now() - m_StartTime;
    }

    Duration lap()
    {
        const TimePoint currentTime = now();
        const Duration elapsed = currentTime - m_StartTime;
        m_StartTime = currentTime;
        return elapsed;
    }

    static TimePoint now()
    {
        return std::chrono::high_resolution_clock::now();
    }

private:
    TimePoint m_StartTime;
}; // Stopwatch
} // BoBRobotics
