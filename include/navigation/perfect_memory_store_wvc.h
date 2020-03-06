#pragma once

// BoB robotics includes
#include "differencers.h"
#include "ridf_processors.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Eigen includes for matrix comparision
#include <Eigen/Dence>
#include <Eigen/Core>

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
// BoBRobotics::Navigation::PerfectMemoryStore::ResponseMatrix
//------------------------------------------------------------------------
/*!
 * \brief The conventional perfect memory (RIDF) algorithm
 *
 * \tparam Differencer This can be AbsDiff or RMSDiff
 */
class ResponseMatrix
{
public:
    ResponseMatrix()
    {
        m_Differencer =  Differencer::FrobeniusDiff();
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getNumSnapshots() const
    {
        return m_Snapshots.size();
    }

    const Eigen::MatrixXd &getSnapshot(size_t index) const
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

    float calcSnapshotDifference(const Eigen::MatrixXd &image, const Eigen::MatrixXd &imageToCompare) const
    {
        // Calculate difference between matrix and stored matrices
        return m_Differencer::mean(image,imageToCompare);

    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<Eigen::MatrixXd> m_Snapshots;
    mutable Differencer m_Differencer;
    mutable cv::Mat m_DiffScratchImage;
}; // RawImage
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
