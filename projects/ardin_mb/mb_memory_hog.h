#pragma once

// Standard C++ includes
#include <tuple>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "navigation/visual_navigation_base.h"

//----------------------------------------------------------------------------
// MBMemoryHOG
//----------------------------------------------------------------------------
class MBMemoryHOG : public BoBRobotics::Navigation::VisualNavigationBase
{
public:
    MBMemoryHOG();
    virtual ~MBMemoryHOG();

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
    void setInput(const std::vector<float> &input);
    std::tuple<unsigned int, unsigned int, unsigned int> present(const cv::Mat &image, bool train) const;

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    cv::HOGDescriptor m_HOG;
    mutable std::vector<float> m_HOGFeatures;

#ifndef CPU_ONLY
    float *m_HOGFeaturesGPU;
#endif
};
