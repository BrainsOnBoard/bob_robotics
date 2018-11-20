#pragma once

// Standard C++ includes
#include <chrono>

namespace BoBRobotics {
class Stopwatch
{
public:
    using time_point = std::chrono::high_resolution_clock::time_point;

    void start()
    {
        m_StartTime = now();
    }

    auto elapsed() const
    {
        return now() - m_StartTime;
    }

    auto lap()
    {
        const auto currentTime = now();
        const auto elapsed = currentTime - m_StartTime;
        m_StartTime = currentTime;
        return elapsed;
    }

    static time_point now()
    {
        return std::chrono::high_resolution_clock::now();
    }

private:
    time_point m_StartTime;
}; // Stopwatch
} // BoBRobotics
