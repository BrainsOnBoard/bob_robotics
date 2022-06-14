#pragma once

// BoB robotics includes
#include "common/stopwatch.h"

// Standard C++ includes
#include <atomic>
#include <string>
#include <thread>

// Standard C includes
#include <cstddef>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::ProgressBar
//----------------------------------------------------------------------------
//! Show the progress of a long-running process including estimated remaining time
class ProgressBar {
public:
    ProgressBar(std::string description, size_t total, size_t initialValue = 0);
    ~ProgressBar();

    //! Cancel the associated thread and reset the console
    void cancel();

    //! Indicate that another op has occurred
    void increment();

private:
    std::thread m_Thread;
    std::string m_Description;
    Stopwatch m_Timer;
    std::atomic<std::chrono::milliseconds> m_Remaining;
    const size_t m_Total;
    std::atomic<size_t> m_Current;
    std::atomic<bool> m_DoRun;

    void run();
}; // ProgressBar
} // BoBRobotics
