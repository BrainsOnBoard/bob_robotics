#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "plog/Log.h"
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

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryStore::HOG
//------------------------------------------------------------------------
//! Perfect memory algorithm using HOG features instead of raw image matching
template<typename Differencer = AbsDiff>
class HOG
{
public:
    HOG(const cv::Size &unwrapRes, const cv::Size &cellSize, int numOrientations)
      : m_HOGDescriptorSize(numOrientations * (unwrapRes.width / cellSize.width)
                            * (unwrapRes.height / cellSize.height))
    {
        LOG_INFO << "Creating perfect memory for " << m_HOGDescriptorSize<< " entry HOG features";

        // Configure HOG features - we want to normalise over the whole image (i.e. one block is the entire image)
        m_HOG.winSize = unwrapRes;
        m_HOG.blockSize = unwrapRes;
        m_HOG.blockStride = unwrapRes;
        m_HOG.cellSize = cellSize;
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
    float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask,
                                 size_t snapshot, const cv::Mat &) const
    {
        BOB_ASSERT(imageMask.empty());

        /*
         * Workaround for a really longstanding gcc bug:
         *      https://gcc.gnu.org/bugzilla/show_bug.cgi?id=27557
         */
        static thread_local typename Differencer::template Internal<std::vector<float>> differencer;

        // Calculate HOG descriptors of image
        auto &scratchDescriptors = differencer.getScratchVector();
        m_HOG.compute(image, scratchDescriptors);
        BOB_ASSERT(scratchDescriptors.size() == m_HOGDescriptorSize);

        // Calculate differences between image HOG descriptors and snapshot
        return differencer(m_Snapshots[snapshot], scratchDescriptors);
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<std::vector<float>> m_Snapshots;
    cv::HOGDescriptor m_HOG;
}; // HOG
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
