#pragma once

// BoB robotics includes
#include "imgproc/mask.h"
#include "navigation/differencers.h"
#include "navigation/ridf_processors.h"

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
    RawImage(const cv::Size &)
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getNumSnapshots() const
    {
        return m_Snapshots.size();
    }

    const std::pair<cv::Mat, ImgProc::Mask> &getSnapshot(size_t index) const
    {
        BOB_ASSERT(index < m_Snapshots.size());
        return m_Snapshots[index];
    }

    size_t addSnapshot(const cv::Mat &image, const ImgProc::Mask &mask)
    {
        // Add a new entry to vector
        m_Snapshots.emplace_back();

        // Make a deep copy of the image
        image.copyTo(m_Snapshots.back().first);

        // Make a shallow copy of the mask
        m_Snapshots.back().second = mask;

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    void clear()
    {
        m_Snapshots.clear();
    }

    float calcSnapshotDifference(const cv::Mat &image,
                                 const ImgProc::Mask &imageMask,
                                 size_t snapshot) const
    {
        static thread_local typename Differencer::template Internal<> differencer;

        // Calculate difference between image and stored image
        return differencer(image, m_Snapshots[snapshot].first, imageMask, m_Snapshots[snapshot].second);
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<std::pair<cv::Mat, ImgProc::Mask>> m_Snapshots;
}; // RawImage
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
