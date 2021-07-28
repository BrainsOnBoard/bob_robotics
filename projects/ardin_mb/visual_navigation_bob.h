#pragma once

// Ardin MB includes
#include "visual_navigation_base.h"

//----------------------------------------------------------------------------
// BoBWrapper
//----------------------------------------------------------------------------
//! Very thin wrapper to allow BoB navigation algorithms
//! which can be used without additional state etc
template<typename T>
class VisualNavigationBoB : public VisualNavigationBase
{
public:
    template<class... Ts>
    VisualNavigationBoB(Ts &&... args) : m_Memory(std::forward<Ts>(args)...)
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
        return m_Memory.test(image);
    }

    //! Perform any updates that should happen at end of test scan
    virtual void resetTestScan() override
    {
    }

    //! Clear the memory
    virtual void clearMemory() override
    {
        m_Memory.clearMemory();
    }

    virtual const cv::Size getUnwrapResolution() const override
    {
        return m_Memory.getUnwrapResolution();
    }

    virtual std::pair<size_t, size_t> getHighlightedWaypoints() const
    {
        return std::make_pair(0, std::numeric_limits<size_t>::max());
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    T m_Memory;
};
