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
#include "differencers.h"
#include "perfect_memory_base.h"
#include "ridf_processors.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryHOG
//------------------------------------------------------------------------
//! Perfect memory algorithm using HOG features instead of raw image matching
template<typename RIDFProcessor = BestMatchingSnapshot,
         typename Differencer = AbsDiff,
         typename Rotater = InSilicoRotater>
class PerfectMemoryHOG : public PerfectMemoryBase<RIDFProcessor, Rotater>
{
public:
    PerfectMemoryHOG(const cv::Size unwrapRes,
                     const int HOGPixelsPerCell = 10,
                     int HOGOrientations = 8)
      : PerfectMemoryBase<RIDFProcessor, Rotater>(unwrapRes)
      , m_HOGDescriptorSize(unwrapRes.width * unwrapRes.height *
                            HOGOrientations / (HOGPixelsPerCell * HOGPixelsPerCell))
      , m_Differencer(m_HOGDescriptorSize)
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

        // Calculate differences between image HOG descriptors and snapshot
        auto diffIter = m_Differencer(m_Snapshots[snapshot], m_ScratchDescriptors, m_ScratchDescriptors);

        // Calculate RMS
        return Differencer::mean(std::accumulate(diffIter, diffIter + m_HOGDescriptorSize, 0.0f),
                                 m_HOGDescriptorSize);
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable std::vector<float> m_ScratchDescriptors;
    std::vector<std::vector<float>> m_Snapshots;
    cv::HOGDescriptor m_HOG;
    mutable Differencer m_Differencer;
}; // PerfectMemoryHOG
} // Navigation
} // BoBRobotics