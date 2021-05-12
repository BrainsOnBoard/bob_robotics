#pragma once

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <utility>

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryWindow {

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::Base
//----------------------------------------------------------------------------
class Base
{
public:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Get window of snapshots
    virtual std::pair<size_t, size_t> getWindow() const = 0;

    //! Updates windows based on index of best snapshot and the corresponding low
    virtual void updateWindow(size_t bestSnapshot, float lowestDifference) = 0;
};

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::Fixed
//----------------------------------------------------------------------------
class Fixed : public Base
{
public:
    Fixed(size_t forwardLookaheadSize, size_t reverseLookaheadSize = 0);

    //------------------------------------------------------------------------
    // Base virtuals
    //------------------------------------------------------------------------
    //! Get window of snapshots
    virtual std::pair<size_t, size_t> getWindow() const override;

    //! Updates windows based on index of best snapshot and the corresponding low
    virtual void updateWindow(size_t bestSnapshot, float) override
    {
        m_MemoryPointer = bestSnapshot;
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    //! How many images to look forward and backwards in perfect memory
    const size_t m_ForwardLookaheadSize;
    const size_t m_ReverseLookaheadSize;

    //! Pointer to last position in window
    size_t m_MemoryPointer;
};

} // PerfectMemoryWindow
} // Navigation
} // BoBRobotics


