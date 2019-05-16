#pragma once

// Standard C++ includes
#include <tuple>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "navigation/visual_navigation_base.h"

//----------------------------------------------------------------------------
// MBMemory
//----------------------------------------------------------------------------
class MBMemory : public BoBRobotics::Navigation::VisualNavigationBase
{
public:
    MBMemory(bool normaliseInput = true);

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) override;

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image) const override;

    //! Clear the memory
    virtual void clearMemory() override;

private:
    std::tuple<unsigned int, unsigned int, unsigned int> present(const cv::Mat &image, bool train) const;

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const bool m_NormaliseInput;

    mutable cv::Mat m_SnapshotFloat;
};
