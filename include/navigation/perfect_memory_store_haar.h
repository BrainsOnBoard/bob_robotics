#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "differencers.h"
#include "ridf_processors.h"

// Third-party includes
#include "plog/Log.h"

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
// BoBRobotics::Navigation::PerfectMemoryStore::Haar
//------------------------------------------------------------------------
//! Perfect memory algorithm using Haar wavelets instead of raw image matching
template<typename Differencer = AbsDiff>
class Haar
{
public:
    Haar(const cv::Size &unwrapRes) 
    :   m_ImageSize(unwrapRes), m_HaarSize(unwrapRes.width / 2, unwrapRes.height / 2)
    {
        // Only even-sized images are supported
        BOB_ASSERT((m_ImageSize.width % 2) == 0);
        BOB_ASSERT((m_ImageSize.height % 2) == 0);
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getNumSnapshots() const
    {
        return m_Snapshots.size();
    }

    const std::pair<cv::Mat, ImgProc::Mask> &getSnapshot(size_t) const
    {
        throw std::runtime_error("When using Haar features, snapshots aren't stored");
    }

    // Add a snapshot to memory and return its index
    size_t addSnapshot(const cv::Mat &image, const ImgProc::Mask &mask)
    {
        BOB_ASSERT(mask.empty());

        m_Snapshots.emplace_back();
        computeHaar(image, m_Snapshots.back());
        
        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    void clear()
    {
        m_Snapshots.clear();
    }

    // Calculate difference between memory and snapshot with index
    float calcSnapshotDifference(const cv::Mat &image,
                                 const ImgProc::Mask &imageMask,
                                 size_t snapshot) const
    {
        BOB_ASSERT(imageMask.empty());

        static thread_local typename Differencer::template Internal<> differencer;

        // Calculate HOG descriptors of image
        auto &scratchHaar = differencer.getScratchVector();
        computeHaar(image, scratchHaar);

        // Calculate differences between image Haar descriptors and snapshot
        return differencer(m_Snapshots[snapshot], scratchHaar);
    }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void downsampleIntoHaar(const cv::Mat &feature, cv::Mat &haar, cv::Point offset) const
    {
        // Loop through rows of feature and ROI
        int featureRow;
        int haarRow;
        for(featureRow = 1, haarRow = offset.y; featureRow < feature.rows; featureRow += 2, haarRow++)
        {
            // Get pointers to rows 
            // **NOTE** start with 2nd pixel of feature row
            const int16_t *featurePixel = feature.ptr<int16_t>(featureRow, 1);
            int16_t *haarPixel = haar.ptr<int16_t>(haarRow, offset.x);

            // Loop through feature pixels in row
            for(int j = 1; j < feature.cols; j += 2, haarPixel++, featurePixel += 2) {
                // Copy into ROI
                *haarPixel = *featurePixel;
            }
        }
    }
    
    void computeHaar(const cv::Mat &image, cv::Mat &haar) const
    {
        // Create thread-local temporaries for computing 4 seperate Haar features
        static thread_local cv::Mat ll(m_HaarSize, CV_16SC1);
        static thread_local cv::Mat hl(m_HaarSize, CV_16SC1);
        static thread_local cv::Mat lh(m_HaarSize, CV_16SC1);
        static thread_local cv::Mat hh(m_HaarSize, CV_16SC1);
        
        // Calculate 4 features
        cv::sepFilter2D(image, ll, CV_16SC1, m_LP, m_LP, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
        cv::sepFilter2D(image, hl, CV_16SC1, m_LP, m_HP, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
        cv::sepFilter2D(image, lh, CV_16SC1, m_HP, m_LP, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
        cv::sepFilter2D(image, hh, CV_16SC1, m_HP, m_HP, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
        
        // Create image to hold Haar features
        haar.create(m_ImageSize, CV_16SC1);

        // Copy 4 features into final image
        downsampleIntoHaar(ll, haar, cv::Point(0, 0));
        downsampleIntoHaar(hl, haar, cv::Point(m_HaarSize.width, 0));
        downsampleIntoHaar(lh, haar, cv::Point(m_HaarSize.width, m_HaarSize.height));
        downsampleIntoHaar(hh, haar, cv::Point(0, m_HaarSize.height));
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const cv::Size m_ImageSize;
    const cv::Size m_HaarSize;
    const std::vector<float> m_LP{1.0f / std::sqrt(2.0f), 1.0f / std::sqrt(2.0f)};
    const std::vector<float> m_HP{-1.0f / std::sqrt(2.0f), 1.0f / std::sqrt(2.0f)};
    std::vector<cv::Mat> m_Snapshots;
}; // Haar
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
