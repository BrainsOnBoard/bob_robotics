#pragma once

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <iostream>
#include <functional>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/units.h"

// BoB robotics includes
#include "../common/assert.h"
#include "../common/image_database.h"

// Local includes
#include "insilico_rotater.h"
#include "visual_navigation_base.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;
using namespace units::dimensionless;
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryBase
//------------------------------------------------------------------------
//! An abstract class which is the base for PerfectMemory and PerfectMemoryHOG
template<typename RIDFProcessor, typename Rotater = InSilicoRotater>
class PerfectMemoryBase
  : public VisualNavigationBase
{
public:
    PerfectMemoryBase(const cv::Size unwrapRes)
      : VisualNavigationBase(unwrapRes)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Return the number of snapshots that have been read into memory
    virtual size_t getNumSnapshots() const = 0;

    //! Return a specific snapshot
    virtual const cv::Mat &getSnapshot(size_t index) const = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);

        // Add snapshot
        addSnapshot(image);
    }

    /*!
     * \brief Get differences between current view and stored snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step and for the AntWorldRotater, one passes in one or more
     * angles.
     */
    template<class... Ts>
    const std::vector<std::vector<float>> &getImageDifferences(Ts &&... args) const
    {
        calcImageDifferences(std::forward<Ts>(args)...);
        return m_Differences;
    }

    /*!
     * \brief Get an estimate for heading based on comparing image with stored
     *        snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step and for the AntWorldRotater, one passes in one or more
     * angles.
     */
    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        calcImageDifferences(std::forward<Ts>(args)...);
        const size_t numSnapshots = getNumSnapshots();

        // Now get the minimum for each snapshot and the column this corresponds to
        std::vector<int> bestColumns(numSnapshots);
        std::vector<float> minDifferences(numSnapshots);
        for (size_t i = 0; i < numSnapshots; i++) {
            const auto elem = std::min_element(std::cbegin(m_Differences[i]), std::cend(m_Differences[i]));
            bestColumns[i] = std::distance(std::cbegin(m_Differences[i]), elem);
            minDifferences[i] = *elem;
        }

        // Return result
        return std::tuple_cat(RIDFProcessor()(getUnwrapResolution(), bestColumns, minDifferences),
                              std::make_tuple(std::cref(m_Differences)));
    }

private:
    template<class... Ts>
    void calcImageDifferences(Ts &&... args) const
    {
        const size_t numSnapshots = getNumSnapshots();
        BOB_ASSERT(numSnapshots > 0);

        // Object for rotating over images
        const cv::Size unwrapRes = getUnwrapResolution();
        Rotater rotater(unwrapRes, getMaskImage(), std::forward<Ts>(args)...);

        // Preallocate snapshot difference vectors
        while (m_Differences.size() < numSnapshots) {
            m_Differences.emplace_back(unwrapRes.width);
        }

        // Scan across image columns
        rotater.rotate(
            [this, numSnapshots](const cv::Mat &fr, const cv::Mat &mask, size_t i)
            {
                // Loop through snapshots
                for (size_t s = 0; s < numSnapshots; s++) {
                    // Calculate difference
                    m_Differences[s][i] = calcSnapshotDifference(fr, mask, s);
                }
            });
    }

    mutable std::vector<std::vector<float>> m_Differences;

protected:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) = 0;

    //! Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const = 0;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics