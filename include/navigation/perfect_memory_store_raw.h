#pragma once

// BoB robotics includes
#include "differencers.h"
#include "ridf_processors.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cassert>
#include <cstdlib>

// Standard C++ includes
#include <numeric>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryStore {

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryStore::RawImage
//------------------------------------------------------------------------
/*!
 * \brief The conventional perfect memory (RIDF) algorithm
 *
 * \tparam Differencer This can be AbsDiff or RMSDiff
 */
template<typename Differencer = AbsDiff>
class RawImage
{
public:
    RawImage(const cv::Size unwrapRes)
      : m_Differencer(unwrapRes.width * unwrapRes.height)
      , m_DiffScratchImage(unwrapRes, CV_8UC1)
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getNumSnapshots() const
    {
        return m_Snapshots.size();
    }

    const cv::Mat &getSnapshot(size_t index) const
    {
        BOB_ASSERT(index < m_Snapshots.size());
        return m_Snapshots[index];
    }

    size_t addSnapshot(const cv::Mat &image)
    {
        m_Snapshots.emplace_back();
        image.copyTo(m_Snapshots.back());

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    void clear()
    {
        m_Snapshots.clear();
    }

    float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot, const cv::Mat &snapshotMask) const
    {
        // Calculate difference between image and stored image
        const int imSize = image.rows * image.cols;
        auto diffIter = m_Differencer(image, m_Snapshots[snapshot], m_DiffScratchImage);

        // If there's no mask
        if (imageMask.empty()) {
            float sumDifference = std::accumulate(diffIter, diffIter + imSize, 0.0f);

            // Return mean
            return Differencer::mean(sumDifference, imSize);
        }
        // Otherwise
        else {
            // Get raw access to rotated mask associated with image and non-rotated mask associated with snapshot
            uint8_t *imageMaskPtr = imageMask.data;
            uint8_t *snapshotMaskPtr = snapshotMask.data;

            // Loop through pixels
            float sumDifference = 0.0f;
            unsigned int numUnmaskedPixels = 0;
            const uint8_t *end = &imageMaskPtr[imSize];
            while (imageMaskPtr < end) {
                // If this pixel is masked by neither of the masks
                if (*imageMaskPtr++ != 0 && *snapshotMaskPtr++) {
                    // Accumulate sum of differences
                    sumDifference += (float) *diffIter;

                    // Increment unmasked pixels count
                    numUnmaskedPixels++;
                }
                diffIter++;
            }

            // Return mean
            return Differencer::mean(sumDifference, (float) numUnmaskedPixels);
        }
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<cv::Mat> m_Snapshots;
    mutable Differencer m_Differencer;
    mutable cv::Mat m_DiffScratchImage;
}; // RawImage
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
