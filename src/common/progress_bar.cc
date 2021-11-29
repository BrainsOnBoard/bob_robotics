// BoB robotics includes
#include "common/macros.h"
#include "common/progress_bar.h"

// Standard C++ includes
#include <chrono>
#include <iostream>

namespace BoBRobotics {
ProgressBar::ProgressBar(std::string description, size_t total, size_t initialValue)
  : m_Description{ std::move(description) }
  , m_Total{ total }
  , m_Current{ initialValue }
  , m_DoRun{ true }
{
    BOB_ASSERT(initialValue <= total);

    std::cout << "[" << initialValue << "/" << m_Total << "]    "
              << m_Description << "..." << std::flush;

    m_Timer.start();
    m_Thread = std::thread{ &ProgressBar::run, this };
}

ProgressBar::~ProgressBar()
{
    cancel();
}

void
ProgressBar::cancel()
{
    if (m_Thread.joinable()) {
        m_DoRun = false;
        m_Thread.join();
    }
}

void
ProgressBar::increment()
{
    const auto cur = ++m_Current;

    // Calculate remaining time
    const auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(m_Timer.elapsed());
    const auto timePerOp = std::chrono::milliseconds(totalTime.count() / cur);
    m_Remaining = (m_Total - cur) * timePerOp;
}

void
ProgressBar::run()
{
    size_t cur = 0;
    do {
        if (cur != 0) {
            std::cout << "\r[" << cur << "/" << m_Total << "]    " << m_Description << " (";

            // Print remaining time in minutes and seconds
            const auto remaining = m_Remaining.load();
            const auto minutes = std::chrono::duration_cast<std::chrono::minutes>(remaining);
            if (minutes > std::chrono::minutes(0)) {
                std::cout << minutes.count() << "m ";
            }

            // Add a bit of whitespace at the end to overwrite previous line's contents
            std::cout << std::chrono::duration_cast<std::chrono::seconds>(remaining - minutes).count()
                      << "s remaining)           " << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cur = m_Current.load();
    } while (m_DoRun && cur < m_Total);

    std::cout << "\r[" << cur << "/" << m_Total << "]    " << m_Description
              << (cur == m_Total ? " completed" : " cancelled") << "            \n";
}

} // BoBRobotics
