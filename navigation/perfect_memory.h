#pragma once

// Standard C includes
#include <cassert>
#include <cstdlib>

// Standard C++ includes
#include <numeric>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/path.h"

// Local includes
#include "differencers.h"
#include "perfect_memory_base.h"
#include "ridf_processors.h"

namespace BoBRobotics {
namespace Navigation {

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemory
//------------------------------------------------------------------------
/*!
 * \brief The conventional perfect memory (RIDF) algorithm
 *
 * \tparam RIDFProcessor The method used to calculate the heading (e.g. single snapshot v. multi-snapshot)
 * \tparam Differencer This can be AbsDiff or RMSDiff
 */
template<typename RIDFProcessor = BestMatchingSnapshot,
         typename Rotater = InSilicoRotater,
         typename Differencer = AbsDiff>
class PerfectMemory
  : public PerfectMemoryBase<RIDFProcessor, Rotater>
{
public:
    PerfectMemory(const cv::Size unwrapRes)
      : PerfectMemoryBase<RIDFProcessor, Rotater>(unwrapRes)
      , m_Differencer(unwrapRes.width * unwrapRes.height)
      , m_DiffScratchImage(unwrapRes, CV_8UC1)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const override
    {
        return m_Snapshots.size();
    }

    virtual const cv::Mat &getSnapshot(size_t index) const override
    {
        assert(index < m_Snapshots.size());
        return m_Snapshots[index];
    }

protected:
    virtual size_t addSnapshot(const cv::Mat &image) override
    {
        m_Snapshots.emplace_back();
        image.copyTo(m_Snapshots.back());

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    virtual float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const override
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
            cv::Mat snapshotMask = this->getMaskImage();
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
}; // PerfectMemory
} // Navigation
} // BoBRobotics