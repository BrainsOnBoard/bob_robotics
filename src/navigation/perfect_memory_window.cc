#include "navigation/perfect_memory_window.h"

// BoB robotics includes
#include "common/macros.h"

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
std::pair<size_t, size_t> clampWindow(size_t memoryPointer, size_t fwdWindowSize,
                                      size_t revWindowSize, size_t numSnapshots)
{
    // If this is our first update, return a full window
    if(memoryPointer == std::numeric_limits<size_t>::max()) {
        return {0, numSnapshots};
    }
    // Otherwise, return a window centred around memory memory pointer
    else {
        // Check memory pointer is valid
        BOB_ASSERT(memoryPointer < numSnapshots);

        // Clamp end of window to last snapshot
        const size_t windowEnd = std::min(memoryPointer + fwdWindowSize, numSnapshots);

        // Clamp beginning of window at first snapshot
        // **NOTE** because size_t is unsigned, we need to be careful at the minimum
        const size_t windowSize = fwdWindowSize + revWindowSize;
        const size_t windowStart = (windowEnd > windowSize) ? (windowEnd - windowSize) : 0;

        // Return window
        return {windowStart, windowEnd};
    }
}
}   // Anonymous namespace

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryWindow {

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::Fixed
//----------------------------------------------------------------------------
Fixed::Fixed(size_t fwdLASize, size_t revLASize)
:   m_FwdWindowSize(fwdLASize), m_RevWindowSize(revLASize),
    m_MemoryPointer(std::numeric_limits<size_t>::max())
{
}
//----------------------------------------------------------------------------
std::pair<size_t, size_t> Fixed::getWindow(size_t numSnapshots) const
{
    // Return valid window, centred around memory pointer
    return clampWindow(m_MemoryPointer, m_FwdWindowSize, m_RevWindowSize, numSnapshots);
}
//----------------------------------------------------------------------------
void Fixed::updateWindow(size_t bestSnapshot, float)
{
    m_MemoryPointer = bestSnapshot;
}
//----------------------------------------------------------------------------
void Fixed::resetWindow()
{
    m_MemoryPointer = std::numeric_limits<size_t>::max();
}

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::DynamicBestMatchGradient
//----------------------------------------------------------------------------
DynamicBestMatchGradient::DynamicBestMatchGradient(size_t fwdWindowSize, const WindowConfig &fwdWindowConfig,
                                                   size_t revWindowSize, const WindowConfig &revWindowConfig)
:   m_FwdWindowConfig(fwdWindowConfig), m_RevWindowConfig(revWindowConfig),
    m_FwdWindowSize(fwdWindowSize), m_RevWindowSize(revWindowSize),
    m_LastLowestDifference(0.0f), m_MemoryPointer(std::numeric_limits<size_t>::max())
{
}
//----------------------------------------------------------------------------
std::pair<size_t, size_t> DynamicBestMatchGradient::getWindow(size_t numSnapshots) const
{
    // Return valid window, centred around memory pointer
    return clampWindow(m_MemoryPointer, m_FwdWindowSize, m_RevWindowSize, numSnapshots);
}
//----------------------------------------------------------------------------
void DynamicBestMatchGradient::updateWindow(size_t bestSnapshot, float lowestDifference)
{
    // If this isn't our first update
    if(m_MemoryPointer != std::numeric_limits<size_t>::max()) {
        // If new difference is lower, decrease fwd and rev lookahead sizes
        // **NOTE** because size_t is unsigned, we need to be careful at the minimum
        if(lowestDifference < m_LastLowestDifference) {
            m_FwdWindowSize = m_FwdWindowConfig.decreaseWindowSize(m_FwdWindowSize);
            m_RevWindowSize = m_RevWindowConfig.decreaseWindowSize(m_RevWindowSize);
        }
        // Otherwise, if new difference is greater, increase both fwd and rev window sizes
        else if(lowestDifference > m_LastLowestDifference){
            m_FwdWindowSize = m_FwdWindowConfig.increaseWindowSize(m_FwdWindowSize);
            m_RevWindowSize = m_RevWindowConfig.increaseWindowSize(m_RevWindowSize);
        }
    }

    // Update memory pointer and last lowest difference
    m_MemoryPointer = bestSnapshot;
    m_LastLowestDifference = lowestDifference;
}
//----------------------------------------------------------------------------
void DynamicBestMatchGradient::resetWindow()
{
    m_MemoryPointer = std::numeric_limits<size_t>::max();
}
} // PerfectMemoryWindow
} // Navigation
} // BoBRobotics
