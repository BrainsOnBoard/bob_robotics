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

// BoB robotics includes
#include "common/rtransform.h"

// Local includes
#include "perfect_memory_base.h"
#include "ridf_processors.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryHOG
//------------------------------------------------------------------------
//! Perfect memory algorithm using HOG features instead of raw image matching
template<typename RIDFProcessor = BestMatchingSnapshot>
class PerfectMemoryHOG : public PerfectMemoryBase<RIDFProcessor>
{
public:
    PerfectMemoryHOG(const cv::Size unwrapRes, const unsigned int scanStep = 1,
                     const filesystem::path outputPath = "snapshots",
                     const int HOGPixelsPerCell = 10, int HOGOrientations = 8)
      : PerfectMemoryBase<RIDFProcessor>(unwrapRes, scanStep, outputPath)
      , m_HOGDescriptorSize(unwrapRes.width * unwrapRes.height *
                            HOGOrientations / (HOGPixelsPerCell * HOGPixelsPerCell))
    {
        std::cout << "Creating perfect memory for HOG features" << std::endl;

        // Configure HOG features
        m_HOG.winSize = unwrapRes;
        m_HOG.blockSize = cv::Size(HOGPixelsPerCell, HOGPixelsPerCell);
        m_HOG.blockStride = m_HOG.blockSize;
        m_HOG.cellSize = m_HOG.blockSize;
        m_HOG.nbins = HOGOrientations;
    }

    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    const unsigned int m_HOGDescriptorSize;

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const override
    {
        return m_Snapshots.size();
    }

    virtual const cv::Mat &getSnapshot(size_t) const override
    {
        throw std::runtime_error("When using HOG features, snapshots aren't stored");
    }

protected:
    // Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) override
    {
        m_Snapshots.emplace_back(m_HOGDescriptorSize);
        m_HOG.compute(image, m_Snapshots.back());
        assert(m_Snapshots.back().size() == m_HOGDescriptorSize);

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    // Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const override
    {
        assert(imageMask.empty());

        // Calculate HOG descriptors of image
        m_HOG.compute(image, m_ScratchDescriptors);
        assert(m_ScratchDescriptors.size() == m_HOGDescriptorSize);

        // Calculate square difference between image HOG descriptors and snapshot
        rtransform(m_Snapshots[snapshot], m_ScratchDescriptors, m_ScratchDescriptors, [](float a, float b) {
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