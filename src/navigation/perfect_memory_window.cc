#include "navigation/perfect_memory_window.h"

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryWindow {

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::Fixed
//----------------------------------------------------------------------------
Fixed::Fixed(size_t forwardLookaheadSize, size_t reverseLookaheadSize)
:   m_ForwardLookaheadSize(forwardLookaheadSize), m_ReverseLookaheadSize(reverseLookaheadSize),
    m_MemoryPointer(std::numeric_limits<size_t>::max())
{
}
//------------------------------------------------------------------------
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
            (m_MemoryPointer >= m_ReverseLookaheadSize) ? (m_MemoryPointer - m_ReverseLookaheadSize) : 0,
            m_MemoryPointer + m_ForwardLookaheadSize);
    }
}
} // PerfectMemoryWindow
} // Navigation
} // BoBRobotics
