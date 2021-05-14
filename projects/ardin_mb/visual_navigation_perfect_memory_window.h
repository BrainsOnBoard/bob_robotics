#pragma once

// Ardin MB includes
#include "visual_navigation_base.h"

// BoB Robotics includes
#include "plog/Log.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_window.h"

// Standard C++ includes
#include <algorithm>

//----------------------------------------------------------------------------
// VisualNavigationPerfectMemoryWindow
//----------------------------------------------------------------------------
//! Very thin wrapper to allow VisualNavigationBase wrap BoB
//! navigation algorithms which can be used without additional state etc
template<typename W, typename S = BoBRobotics::Navigation::PerfectMemoryStore::RawImage<>>
class VisualNavigationPerfectMemoryWindow : public VisualNavigationBase
{
    typedef BoBRobotics::Navigation::PerfectMemory<S> PerfectMemory;

public:
    template<typename... Ps, typename... Ws>
    VisualNavigationPerfectMemoryWindow(const std::tuple<Ps...> &pmArgs, const std::tuple<Ws...> &windowArgs)
    :   m_Memory(construct<PerfectMemory>(pmArgs, std::index_sequence_for<Ps...>{})),
        m_Window(construct<W>(windowArgs, std::index_sequence_for<Ws...>{})),
        m_MinTestDifference(std::numeric_limits<float>::max()), m_BestImageIndex(std::numeric_limits<size_t>::max())
    {
    }

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) override
    {
        m_Memory.train(image);
    }

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image) override
    {
        // Get window
        const auto window = m_Window.getWindow();

        LOGD << "Testing within window" << window.first << "-" << window.second;

        // Get image differences within window
        const auto &differences = m_Memory.getImageDifferences(window, image);

        // Find minimum difference in window
        const auto minDifference = std::min_element(differences.begin(), differences.end());

        // If this is better than current minimum, update minimum difference and best image index
        // **NOTE** getImageDifferences only returns differences within window so we need to add on start index of window
        if(*minDifference < m_MinTestDifference) {
            m_MinTestDifference = *minDifference;
            m_BestImageIndex = window.first + std::distance(differences.begin(), minDifference);
        }

        // Return minimum difference
        return *minDifference;
    }

    //! Perform any updates that should happen at end of test scan
    virtual void resetTestScan() override
    {
        LOGD << "Updating window with best image=" << m_BestImageIndex << ", difference:" << m_MinTestDifference;

        // Update window
        m_Window.updateWindow(m_BestImageIndex, m_MinTestDifference);

        // Reset state
        m_MinTestDifference = std::numeric_limits<float>::max();
        m_BestImageIndex = std::numeric_limits<size_t>::max();
    }

    //! Clear the memory
    virtual void clearMemory() override
    {
        // **TODO** clear window
        m_Memory.clearMemory();
    }

    virtual const cv::Size getUnwrapResolution() const override
    {
        return m_Memory.getUnwrapResolution();
    }

    virtual std::pair<size_t, size_t> getHighlightedWaypoints() const
    {
        return m_Window.getWindow();
    }

private:
    //------------------------------------------------------------------------
    // Static helpers
    //------------------------------------------------------------------------
    //! Without std::apply in C++17 one step of indirection is required to call
    //! functions (and constructors) with a tuple of arguments...This is that!
    template<typename T, typename... As, std::size_t... Is>
    static T construct(const std::tuple<As...> &args, std::index_sequence<Is...>)
    {
        return T(std::get<Is>(args)...);
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    PerfectMemory m_Memory;
    W m_Window;

    float m_MinTestDifference;
    size_t m_BestImageIndex;
};
