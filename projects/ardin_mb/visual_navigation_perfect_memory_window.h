#pragma once

// Ardin MB includes
#include "visual_navigation_base.h"

// BoB Robotics includes
#include "plog/Log.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_window.h"

// Standard C++ includes
#include <algorithm>
#include <memory>

//----------------------------------------------------------------------------
// VisualNavigationPerfectMemoryWindow
//----------------------------------------------------------------------------
//! Very thin wrapper to allow VisualNavigationBase wrap BoB
//! navigation algorithms which can be used without additional state etc
template<typename S = BoBRobotics::Navigation::PerfectMemoryStore::RawImage<>>
class VisualNavigationPerfectMemoryWindow : public VisualNavigationBase
{
    template<typename T>
    using PerfectMemory = BoBRobotics::Navigation::PerfectMemory<T>;

    typedef std::unique_ptr<BoBRobotics::Navigation::PerfectMemoryWindow::Base> WindowPtr;

public:
    template<class... Ts>
    VisualNavigationPerfectMemoryWindow(WindowPtr window, Ts &&... args)
    :   m_Memory(std::forward<Ts>(args)...), m_Window(std::move(window)),
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
        const auto window = m_Window->getWindow();

        LOGD << "Testing within window" << window.first << "-" << window.second;

        // Get image differences within window
        const auto &differences = m_Memory.getImageDifferences(image, window);

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
        m_Window->updateWindow(m_BestImageIndex, m_MinTestDifference);

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

    virtual const cv::Size &getUnwrapResolution() const override
    {
        return m_Memory.getUnwrapResolution();
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    PerfectMemory<S> m_Memory;
    WindowPtr m_Window;

    float m_MinTestDifference;
    size_t m_BestImageIndex;
};
