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
    RawImage(const cv::Size &)
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
        /*
         * Workaround for a really longstanding gcc bug:
         *      https://gcc.gnu.org/bugzilla/show_bug.cgi?id=27557
         */
        static thread_local typename Differencer::template Internal<> differencer;

        // Calculate difference between image and stored image
        return differencer(image, m_Snapshots[snapshot], imageMask, snapshotMask);
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<cv::Mat> m_Snapshots;
}; // RawImage
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
