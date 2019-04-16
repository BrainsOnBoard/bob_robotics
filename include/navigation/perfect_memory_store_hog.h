#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/logging.h"
#include "differencers.h"
#include "ridf_processors.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryStore {

    /*numOrientations*
        ((unwrapRes.width - blockSize.width)/blockStride.width + 1)*
        ((unwrapRes.height - blockSize.height)/blockStride.height + 1);*/
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryStore::HOG
//------------------------------------------------------------------------
//! Perfect memory algorithm using HOG features instead of raw image matching
template<typename Differencer = AbsDiff>
class HOG
{
public:
    HOG(const cv::Size &unwrapRes, const cv::Size &blockSize, int numOrientations)
    :   HOG(unwrapRes, blockSize, blockSize, numOrientations)
    {
    }

    HOG(const cv::Size &unwrapRes, const cv::Size &blockSize, const cv::Size &blockStride, int numOrientations)
    :   m_HOGDescriptorSize(numOrientations * ((unwrapRes.width - blockSize.width)/blockStride.width + 1) * ((unwrapRes.height - blockSize.height)/blockStride.height + 1)),
        m_Differencer(m_HOGDescriptorSize)
    {
        LOG_INFO << "Creating perfect memory for " << m_HOGDescriptorSize<< " entry HOG features";

        // Configure HOG features
        m_HOG.winSize = unwrapRes;
        m_HOG.blockSize = blockSize;
        m_HOG.blockStride = blockStride;
        m_HOG.cellSize = blockSize;
        m_HOG.nbins = numOrientations;
    }

    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    const unsigned int m_HOGDescriptorSize;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getNumSnapshots() const
    {
        return m_Snapshots.size();
    }

    const cv::Mat &getSnapshot(size_t) const
    {
        throw std::runtime_error("When using HOG features, snapshots aren't stored");
    }

    // Add a snapshot to memory and return its index
    size_t addSnapshot(const cv::Mat &image)
    {
        m_Snapshots.emplace_back(m_HOGDescriptorSize);
        m_HOG.compute(image, m_Snapshots.back());
        BOB_ASSERT(m_Snapshots.back().size() == m_HOGDescriptorSize);

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    void clear()
    {
        m_Snapshots.clear();
    }

    // Calculate difference between memory and snapshot with index
    float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot, const cv::Mat &) const
    {
        BOB_ASSERT(imageMask.empty());

        // Calculate HOG descriptors of image
        m_HOG.compute(image, m_ScratchDescriptors);
        BOB_ASSERT(m_ScratchDescriptors.size() == m_HOGDescriptorSize);

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
}; // HOG
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
