#pragma once

// Standard C++ includes
#include <bitset>
#include <list>
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
    // Typedefines
    //------------------------------------------------------------------------
    typedef std::list<std::pair<double, std::vector<unsigned int>>> Spikes;

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) override;

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image) const override;

    //! Clear the memory
    virtual void clearMemory() override;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    const std::vector<float> &getHOGFeatures() const{ return m_HOGFeatures; }

    const Spikes &getPNSpikes() const{ return m_PNSpikes; }
    const Spikes &getKCSpikes() const{ return m_KCSpikes; }
    const Spikes &getENSpikes() const{ return m_ENSpikes; }

    unsigned int getNumUnusedWeights() const{ return m_NumUsedWeights; }
    unsigned int getNumActivePN() const{ return m_NumActivePN; }
    unsigned int getNumActiveKC() const{ return m_NumActiveKC; }

private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
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

    mutable Spikes m_PNSpikes;
    mutable Spikes m_KCSpikes;
    mutable Spikes m_ENSpikes;

    mutable unsigned int m_NumUsedWeights;
    mutable unsigned int m_NumActivePN;
    mutable unsigned int m_NumActiveKC;
};
