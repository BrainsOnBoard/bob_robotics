#pragma once

// Standard C++ includes
#include <numeric>
#include <vector>

// Standard C includes
#include <cassert>
#include <cstdlib>

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
template<typename RIDFProcessor = BestMatchingSnapshot, typename Differencer = AbsDiff>
class PerfectMemory
  : public PerfectMemoryBase<RIDFProcessor>
{
public:
    PerfectMemory(const cv::Size unwrapRes, const unsigned int scanStep = 1, const filesystem::path outputPath = "snapshots")
      : PerfectMemoryBase<RIDFProcessor>(unwrapRes, scanStep, outputPath)
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
        m_Differencer.calculateDifference(image, m_Snapshots[snapshot], m_DiffScratchImage);

        // If there's no mask
        if (imageMask.empty()) {
            float sumDifference = std::accumulate(std::cbegin(m_Differencer), std::cend(m_Differencer), 0.0f);

            // Return mean
            return Differencer::mean(sumDifference, m_DiffScratchImage.cols * m_DiffScratchImage.rows);
        }
        // Otherwise
        else {
            // Get raw access to rotated mask associated with image and non-rotated mask associated with snapshot
            const uint8_t *rawImageMask = reinterpret_cast<const uint8_t *>(imageMask.data);
            const uint8_t *rawSnapshotMask = reinterpret_cast<const uint8_t *>(this->getMaskImage().data);
            const auto differences = std::cbegin(m_Differencer);

            // Loop through pixels
            float sumDifference = 0.0f;
            unsigned int numUnmaskedPixels = 0;
            for (int i = 0; i < (m_DiffScratchImage.cols * m_DiffScratchImage.rows); i++) {
                // If this pixel is masked by neither of the masks
                if (rawImageMask[i] != 0 && rawSnapshotMask[i] != 0) {
                    // Accumulate sum of differences
                    sumDifference += (float) differences[i];

                    // Increment unmasked pixels count
                    numUnmaskedPixels++;
                }
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