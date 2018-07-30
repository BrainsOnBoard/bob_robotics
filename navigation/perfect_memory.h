#pragma once

// Standard C++ includes
#include <iostream>
#include <vector>

// Standard C includes
#include <cassert>
#include <cstdlib>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Third-party includes
#include "third_party/path.h"

// Local includes
#include "perfect_memory_base.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemory
//------------------------------------------------------------------------
template<unsigned int scanStep>
class PerfectMemory : public PerfectMemoryBase<scanStep>
{
public:
    PerfectMemory(const cv::Size unwrapRes, const std::string outputPath = "snapshots",
                  const std::string filenamePrefix = "snapshot_")
      : PerfectMemoryBase<scanStep>(unwrapRes, outputPath, filenamePrefix)
      , m_DiffScratchImage(unwrapRes, CV_8UC1)
    {
        std::cout << "Creating perfect memory for raw images" << std::endl;
    }

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const override { return m_Snapshots.size(); }
    virtual const cv::Mat &getSnapshot(size_t index) const override{ return m_Snapshots[index]; }

protected:
    // Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) override
    {
        m_Snapshots.emplace_back();
        image.copyTo(m_Snapshots.back());

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    // Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifferenceSquared(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const override
    {
        // Calculate absolute difference between image and stored image
        cv::absdiff(m_Snapshots[snapshot], image, m_DiffScratchImage);

        // Get raw access to image difference values
        const uint8_t *rawDiff = reinterpret_cast<const uint8_t*>(m_DiffScratchImage.data);

        // If there's no mask
        if(imageMask.empty()) {
            // Loop through pixels
            float sumSqrDifference = 0.0f;
            for(int i = 0; i < (m_DiffScratchImage.cols * m_DiffScratchImage.rows); i++) {
                // Accumulate sum of squared differences
                const float diff = (float)rawDiff[i];
                sumSqrDifference += (diff * diff);
            }

            // Scale down by number of pixels and take square root
            return sqrt(sumSqrDifference / (float)(m_DiffScratchImage.cols * m_DiffScratchImage.rows));
        }
        // Otherwise
        else {
            // Get raw access to rotated mask associated with image and non-rotated mask associated with snapshot
            const uint8_t *rawImageMask = reinterpret_cast<const uint8_t*>(imageMask.data);
            const uint8_t *rawSnapshotMask = reinterpret_cast<const uint8_t*>(this->getMaskImage().data);

            // Loop through pixels
            float sumSqrDifference = 0.0f;
            unsigned int numUnmaskedPixels = 0;
            for(int i = 0; i < (m_DiffScratchImage.cols * m_DiffScratchImage.rows); i++) {
                // If this pixel is masked by neither of the masks
                if(rawImageMask[i] != 0 && rawSnapshotMask[i] != 0) {
                    // Accumulate sum of squared differences
                    const float diff = (float)rawDiff[i];
                    sumSqrDifference += (diff * diff);

                    // Increment unmasked pixels count
                    numUnmaskedPixels++;
                }
            }

            // Scale down by number of unmasked pixels and take square root
            return sqrt(sumSqrDifference / (float)numUnmaskedPixels);
        }
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<cv::Mat> m_Snapshots;
    mutable cv::Mat m_DiffScratchImage;
}; // PerfectMemory
} // Navigation
} // BoBRobotics