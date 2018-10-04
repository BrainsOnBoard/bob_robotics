#pragma once

// Standard C includes
#include <cassert>
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

// Local includes
#include "differencers.h"
#include "insilico_rotater.h"
#include "perfect_memory_store_raw.h"
#include "visual_navigation_base.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;
using namespace units::dimensionless;
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemory
//------------------------------------------------------------------------
template<typename Store = PerfectMemoryStore::RawImage<>>
class PerfectMemory : public VisualNavigationBase
{
public:
    PerfectMemory(const cv::Size unwrapRes /**TODO*** forward other parameters*/) : VisualNavigationBase(unwrapRes), m_Store(unwrapRes)
    {}

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        // Add snapshot
        m_Store.addSnapshot(image);
    }

    virtual float test(const cv::Mat &image) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        const size_t numSnapshots = getNumSnapshots();
        assert(numSnapshots > 0);

        // Clear differences (won't free)
        m_Differences.clear();

        // Loop through snapshots and caculate differences
        for (size_t s = 0; s < numSnapshots; s++) {
            m_Differences.push_back(calcSnapshotDifference(image, getMaskImage(), s));
        }

        // Return smallest difference
        return *std::min_element(m_Differences.begin(), m_Differences.end());
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
     //! Return the number of snapshots that have been read into memory
    size_t getNumSnapshots() const{ return m_Store.getNumSnapshots(); }

    //! Return a specific snapshot
    const cv::Mat &getSnapshot(size_t index) const{ return m_Store.getSnapshot(index); }

    /*!
     * \brief Get differences between current view and stored snapshots
     */
    const std::vector<float> &getImageDifferences(const cv::Mat &image) const
    {
        test(image);
        return m_Differences;
    }

protected:
    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const
    {
        return m_Store.calcSnapshotDifference(image, imageMask, snapshot, getMaskImage());
    }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    Store m_Store;
    mutable std::vector<float> m_Differences;
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryRotater
//------------------------------------------------------------------------
template<typename Store = PerfectMemoryStore::RawImage<>, typename RIDFProcessor = BestMatchingSnapshot, typename Rotater = InSilicoRotater>
class PerfectMemoryRotater : public PerfectMemory<Store>
{
public:
    PerfectMemoryRotater(const cv::Size unwrapRes /**TODO*** forward other parameters*/) : PerfectMemory<Store>(unwrapRes)
    {
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
        return m_RotatedDifferences;
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
        const size_t numSnapshots = this->getNumSnapshots();

        // Now get the minimum for each snapshot and the column this corresponds to
        std::vector<int> bestColumns(numSnapshots);
        std::vector<float> minDifferences(numSnapshots);
        for (size_t i = 0; i < numSnapshots; i++) {
            const auto elem = std::min_element(std::cbegin(m_RotatedDifferences[i]), std::cend(m_RotatedDifferences[i]));
            bestColumns[i] = std::distance(std::cbegin(m_RotatedDifferences[i]), elem);
            minDifferences[i] = *elem;
        }

        // Return result
        return std::tuple_cat(RIDFProcessor()(this->getUnwrapResolution(), bestColumns, minDifferences),
                              std::make_tuple(std::cref(m_RotatedDifferences)));
    }

private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<class... Ts>
    void calcImageDifferences(Ts &&... args) const
    {
        const size_t numSnapshots = this->getNumSnapshots();
        assert(numSnapshots > 0);

        // Object for rotating over images
        const cv::Size unwrapRes = this->getUnwrapResolution();
        Rotater rotater(unwrapRes, this->getMaskImage(), std::forward<Ts>(args)...);

        // Preallocate snapshot difference vectors
        while (m_RotatedDifferences.size() < numSnapshots) {
            m_RotatedDifferences.emplace_back(unwrapRes.width);
        }

        // Scan across image columns
        rotater.rotate(
            [this, numSnapshots](const cv::Mat &fr, const cv::Mat &mask, size_t i)
            {
                // Loop through snapshots
                for (size_t s = 0; s < numSnapshots; s++) {
                    // Calculate difference
                    m_RotatedDifferences[s][i] = this->calcSnapshotDifference(fr, mask, s);
                }
            });
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable std::vector<std::vector<float>> m_RotatedDifferences;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics