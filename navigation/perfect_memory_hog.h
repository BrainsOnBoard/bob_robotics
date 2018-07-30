#pragma once

// Standard C includes
#include <cassert>
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <stdexcept>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// Local includes
#include "config.h"
#include "perfect_memory_base.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryHOG
//------------------------------------------------------------------------
template<unsigned int scanStep>
class PerfectMemoryHOG : public PerfectMemoryBase<scanStep>
{
public:
    PerfectMemoryHOG(const Config &config)
    :   PerfectMemoryBase<scanStep>(config),
        HOGDescriptorSize(config.getHOGDescriptorSize()),
        m_ScratchDescriptors(HOGDescriptorSize)
    {
        std::cout << "Creating perfect memory for HOG features" << std::endl;
        
        // Configure HOG features
        m_HOG.winSize = config.getUnwrapRes(); 
        m_HOG.blockSize = cv::Size(config.getNumHOGPixelsPerCell(), config.getNumHOGPixelsPerCell());
        m_HOG.blockStride = cv::Size(config.getNumHOGPixelsPerCell(), config.getNumHOGPixelsPerCell());
        m_HOG.cellSize = cv::Size(config.getNumHOGPixelsPerCell(), config.getNumHOGPixelsPerCell());
        m_HOG.nbins = config.getNumHOGOrientations();
    }

    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    const unsigned int HOGDescriptorSize;

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const override { return m_Snapshots.size(); }
    virtual const cv::Mat &getSnapshot(size_t index) const override{ throw std::runtime_error("When using HOG features, snapshots aren't stored"); }

protected:
    // Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) override
    {
        m_Snapshots.emplace_back(HOGDescriptorSize);
        m_HOG.compute(image, m_Snapshots.back());
        assert(m_Snapshots.back().size() == HOGDescriptorSize);

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    // Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifferenceSquared(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const override
    {
        assert(imageMask.empty());

        // Calculate HOG descriptors of image
        m_HOG.compute(image, m_ScratchDescriptors);
        assert(m_ScratchDescriptors.size() == HOGDescriptorSize);

        // Calculate square difference between image HOG descriptors and snapshot
        std::transform(m_Snapshots[snapshot].begin(), m_Snapshots[snapshot].end(),
                       m_ScratchDescriptors.begin(), m_ScratchDescriptors.begin(),
                       [](float a, float b)
                       {
                           return (a - b) * (a - b);
                       });

        // Calculate RMS
        return sqrt(std::accumulate(m_ScratchDescriptors.begin(), m_ScratchDescriptors.end(), 0.0f));
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable std::vector<float> m_ScratchDescriptors;
    std::vector<std::vector<float>> m_Snapshots;
    cv::HOGDescriptor m_HOG;
}; // PerfectMemoryHOG
} // Navigation
} // BoBRobotics