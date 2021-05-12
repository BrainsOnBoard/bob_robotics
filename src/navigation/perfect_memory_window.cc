#include "navigation/perfect_memory_window.h"

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryWindow {

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::Fixed
//----------------------------------------------------------------------------
Fixed::Fixed(size_t fwdLASize, size_t revLASize)
:   m_FwdLASize(fwdLASize), m_RevLASize(revLASize),
    m_MemoryPointer(std::numeric_limits<size_t>::max())
{
}
//----------------------------------------------------------------------------
std::pair<size_t, size_t> Fixed::getWindow() const
{
    // If this is our first update, return a full window
    if(m_MemoryPointer == std::numeric_limits<size_t>::max()) {
        return std::make_pair(0, std::numeric_limits<size_t>::max());
    }
    // Otherwise, return a window centred around memory memory pointer
    // **NOTE** because size_t is unsigned, we need to be careful at the minimum
    else {
        return std::make_pair(
            (m_MemoryPointer >= m_RevLASize) ? (m_MemoryPointer - m_RevLASize) : 0,
            m_MemoryPointer + m_FwdLASize);
    }
}

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::DynamicBestMatchGradient
//----------------------------------------------------------------------------
DynamicBestMatchGradient::DynamicBestMatchGradient(size_t fwdLASize, size_t fwdLAIncreaseSize, size_t fwdLADecreaseSize, size_t minFwdLASize, size_t maxFwdLASize,
                                                   size_t revLASize, size_t revLAIncreaseSize, size_t revLADecreaseSize, size_t minRevLASize, size_t maxRevLASize)
:   m_MinFwdLASize(minFwdLASize), m_MinRevLASize(minRevLASize),
    m_MaxFwdLASize(maxFwdLASize), m_MaxRevLASize(maxRevLASize),
    m_FwdLAIncreaseSize(fwdLAIncreaseSize), m_RevLAIncreaseSize(revLAIncreaseSize),
    m_FwdLADecreaseSize(fwdLADecreaseSize), m_RevLADecreaseSize(revLADecreaseSize),
    m_FwdLASize(fwdLASize), m_RevLASize(revLASize),
    m_LastLowestDifference(0.0f), m_MemoryPointer(std::numeric_limits<size_t>::max())
{
}
//----------------------------------------------------------------------------
std::pair<size_t, size_t> DynamicBestMatchGradient::getWindow() const
{
    // If this is our first update, return a full window
    if(m_MemoryPointer == std::numeric_limits<size_t>::max()) {
        return std::make_pair(0, std::numeric_limits<size_t>::max());
    }
    // Otherwise, return a window centred around memory memory pointer
    // **NOTE** because size_t is unsigned, we need to be careful at the minimum
    else {
        return std::make_pair(
            (m_MemoryPointer >= m_RevLASize) ? (m_MemoryPointer - m_RevLASize) : 0,
            m_MemoryPointer + m_FwdLASize);
    }
}
//----------------------------------------------------------------------------
void DynamicBestMatchGradient::updateWindow(size_t bestSnapshot, float lowestDifference)
{
    // If this isn't our first update
    if(m_MemoryPointer != std::numeric_limits<size_t>::max()) {
        // If new difference is lower, decrease fwd and rev lookahead sizes
        // **NOTE** because size_t is unsigned, we need to be careful at the minimum
        if(lowestDifference < m_LastLowestDifference) {
            m_FwdLASize = (m_FwdLASize > (m_MinFwdLASize + m_FwdLADecreaseSize)) ? (m_FwdLASize - m_FwdLADecreaseSize) : m_MinFwdLASize;
            m_RevLASize = (m_RevLASize > (m_MinRevLASize + m_RevLADecreaseSize)) ? (m_RevLASize - m_RevLADecreaseSize) : m_MinRevLASize;
        }
        // Otherwise, if new difference is greater, increase both fwd and rev lookahead sizes
        else if(lowestDifference > m_LastLowestDifference){
            m_FwdLASize = std::min(m_MaxFwdLASize,
                                              m_FwdLASize + m_FwdLAIncreaseSize);
            m_RevLASize = std::min(m_MaxRevLASize,
                                              m_RevLASize + m_RevLAIncreaseSize);
        }
    }

    // Update memory pointer and last lowest difference
    m_MemoryPointer = bestSnapshot;
    m_LastLowestDifference = lowestDifference;
}
} // PerfectMemoryWindow
} // Navigation
} // BoBRobotics
