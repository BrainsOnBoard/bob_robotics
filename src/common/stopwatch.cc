// BoB robotics includes
#include "common/macros.h"
#include "common/stopwatch.h"

namespace BoBRobotics {
void
Stopwatch::start()
{
    m_StartTime = now();
}

bool
Stopwatch::started() const
{
    return m_StartTime != TimePoint::min();
}

void
Stopwatch::reset()
{
    m_StartTime = TimePoint::min();
}

Stopwatch::Duration
Stopwatch::elapsed() const
{
    BOB_ASSERT(started());
    return now() - m_StartTime;
}

Stopwatch::Duration
Stopwatch::lap()
{
    const TimePoint currentTime = now();
    const Duration elapsed = currentTime - m_StartTime;
    m_StartTime = currentTime;
    return elapsed;
}

Stopwatch::TimePoint
Stopwatch::now()
{
    return std::chrono::high_resolution_clock::now();
}
} // BoBRobotics
