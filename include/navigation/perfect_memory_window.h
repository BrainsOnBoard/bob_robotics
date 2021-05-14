#pragma once

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <algorithm>
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

    //! Resets window e.g. if agent is displaced
    virtual void resetWindow() = 0;
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
    virtual void updateWindow(size_t bestSnapshot, float) override;

    //! Resets window e.g. if agent is displaced
    virtual void resetWindow() override;

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
    struct WindowConfig
    {
        //! How much to increase the size of window by if best image difference increases
        size_t increaseSize;

        //! How much to decrease the size of window by if best image difference decreases
        size_t decreaseSize;

         //! Minimum size of window to search in perfect memory
        size_t minSize;

        //! Maximum size of window to search in perfect memory
        size_t maxSize;

        //! Helper to decrease size as specified by config
        size_t decreaseWindowSize(size_t size) const
        {
            return (size > (minSize + decreaseSize)) ? (size - decreaseSize) : minSize;
        }

        //! Helper to increase size as specified by config
        size_t increaseWindowSize(size_t size) const
        {
            return std::min(maxSize, size + increaseSize);
        }
    };

    DynamicBestMatchGradient(size_t fwdWindowSize, const WindowConfig &fwdWindowConfig,
                             size_t revWindowSize = 0, const WindowConfig &revWindowConfig = {0, 0, 0, 0});

    //------------------------------------------------------------------------
    // Base virtuals
    //------------------------------------------------------------------------
    //! Get window of snapshots
    virtual std::pair<size_t, size_t> getWindow() const override;

    //! Updates windows based on index of best snapshot and the corresponding low
    virtual void updateWindow(size_t bestSnapshot, float lowestDifference) override;

    //! Resets window e.g. if agent is displaced
    virtual void resetWindow() override;

private:


    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Configuration of forward and reverse search windows
    const WindowConfig m_FwdWindowConfig;
    const WindowConfig m_RevWindowConfig;

    //! How many images to look fwd and backwards in perfect memory
    size_t m_FwdWindowSize;
    size_t m_RevWindowSize;

    //! Lowest image difference encountered at last update
    float m_LastLowestDifference;

    //! Pointer to last position in window
    size_t m_MemoryPointer;
};
} // PerfectMemoryWindow
} // Navigation
} // BoBRobotics


