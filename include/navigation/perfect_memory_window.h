#pragma once

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <limits>
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
// BoBRobotics::Navigation::PerfectMemoryWindow::Full
//----------------------------------------------------------------------------
//! Full window always searches entire perfect memory
class Full : public Base
{
public:
    //------------------------------------------------------------------------
    // Base virtuals
    //------------------------------------------------------------------------
    //! Get window of snapshots
    virtual std::pair<size_t, size_t> getWindow() const override
    {
        return std::make_pair(0, std::numeric_limits<size_t>::max());
    }

    //! Updates windows based on index of best snapshot and the corresponding low
    virtual void updateWindow(size_t, float) override
    {
    }
};

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::Fixed
//----------------------------------------------------------------------------
//! Fixed windows searches for route images within a fixed size window
class Fixed : public Base
{
public:
    Fixed(size_t fwdLASize, size_t revLASize = 0);

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
    //! How many images to look fwd and backwards in perfect memory
    const size_t m_FwdLASize;
    const size_t m_RevLASize;

    //! Pointer to last position in window
    size_t m_MemoryPointer;
};

//----------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWindow::DynamicBestMatchGradient
//----------------------------------------------------------------------------
//! Dynamic best match gradient windows searches for route images within a window
//! which grows and contracts based on whether familiarity increases or decreases
class DynamicBestMatchGradient : public Base
{
public:
    DynamicBestMatchGradient(size_t fwdLASize, size_t fwdLAIncreaseSize, size_t fwdLADecreaseSize, size_t minFwdLASize, size_t maxFwdLASize,
                             size_t revLASize = 0, size_t revLAIncreaseSize = 0, size_t revLADecreaseSize = 0, size_t minRevLASize = 0, size_t maxRevLASize = 0);

    //------------------------------------------------------------------------
    // Base virtuals
    //------------------------------------------------------------------------
    //! Get window of snapshots
    virtual std::pair<size_t, size_t> getWindow() const override;

    //! Updates windows based on index of best snapshot and the corresponding low
    virtual void updateWindow(size_t bestSnapshot, float lowestDifference) override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    //! Minimum numbers of images to look fwd and backwards in perfect memory
    const size_t m_MinFwdLASize;
    const size_t m_MinRevLASize;

    //! Maximums numbers of images to look fwd and backwards in perfect memory
    const size_t m_MaxFwdLASize;
    const size_t m_MaxRevLASize;

    //! How much to increase fwd and rev lookaheads by if difference increases
    const size_t m_FwdLAIncreaseSize;
    const size_t m_RevLAIncreaseSize;

    //! How much to decrease fwd and rev lookaheads by if difference decreases
    const size_t m_FwdLADecreaseSize;
    const size_t m_RevLADecreaseSize;

    //! How many images to look fwd and backwards in perfect memory
    size_t m_FwdLASize;
    size_t m_RevLASize;

    float m_LastLowestDifference;

    //! Pointer to last position in window
    size_t m_MemoryPointer;
};
} // PerfectMemoryWindow
} // Navigation
} // BoBRobotics


