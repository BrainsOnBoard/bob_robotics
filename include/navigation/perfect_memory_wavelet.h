#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "differencers.h"
#include "insilico_rotater.h"
#include "perfect_memory_store_wvc.h"
#include "visual_navigation_base.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryWavelet
//------------------------------------------------------------------------
template<typename Store = PerfectMemoryWaveletStore::RawImage<>>
class PerfectMemoryWavelet : public VisualNavigationBase
{
public:
    template<class... Ts>
    PerfectMemoryWavelet(const cv::Size &unwrapRes, Ts &&... args)
      : VisualNavigationBase(unwrapRes)
      , m_Store(unwrapRes, std::forward<Ts>(args)...)
    {}

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    virtual void train(const Eigen::MatrixXd &image) override
    {
        // Add snapshot
        m_Store.addSnapshot(image);
    }

    virtual float test(const Eigen::MatrixXd &image &image) const override
    {
        const size_t numSnapshots = getNumSnapshots();
        BOB_ASSERT(numSnapshots > 0);

        // Clear differences (won't free)
        m_Differences.clear();

        // Loop through snapshots and caculate differences
        for (size_t s = 0; s < numSnapshots; s++) {
            m_Differences.push_back(calcSnapshotDifference(image, getMaskImage(), s));
        }

        // Return smallest difference
        return *std::min_element(m_Differences.begin(), m_Differences.end());
    }

    virtual void clearMemory() override
    {
        m_Store.clear();
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
     //! Return the number of snapshots that have been read into memory
    size_t getNumSnapshots() const{ return m_Store.getNumSnapshots(); }

    //! Return a specific snapshot
    const Eigen::MatrixXd &getSnapshot(size_t index) const override{ return m_Store.getSnapshot(index); }

    /*!
     * \brief Get differences between current view and stored snapshots
     */
    const std::vector<float> &getImageDifferences(const Eigen::MatrixXd &image) const override
    {
        test(image);
        return m_Differences;
    }

protected:
    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    float calcSnapshotDifference(const Eigen::MatrixXd &image, const Eigen::MatrixXd &imageMask, size_t snapshot) const override
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
// TODO create right rotator
template<typename Store = PerfectMemoryStore::ResponseMatrix<>, typename RIDFProcessor = BestMatchingSnapshot, typename Rotater = InSilicoRotater>
class PerfectMemoryRotater : public PerfectMemory<Store>
{
public:
    template<class... Ts>
    PerfectMemoryRotater(const cv::Size unwrapRes, Ts &&... args)
    :   PerfectMemory<Store>(unwrapRes, std::forward<Ts>(args)...)
    {
    }

    /*!
     * \brief Get differences between current view and stored snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a Eigen::MatrixXd and (optionally) an unsigned int
     * for the scan step and for the AntWorldRotater, one passes in one or more
     * angles.
     */
    template<class... Ts>
    const std::vector<std::vector<float>> &getImageDifferences(Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), this->getMaskImage(), std::forward<Ts>(args)...);
        calcImageDifferences(rotater);
        return m_RotatedDifferences;
    }

    /*!
     * \brief Get an estimate for heading based on comparing image with stored
     *        snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a Eigen::MatrixXd and (optionally) an unsigned int
     * for the scan step and for the AntWorldRotater, one passes in one or more
     * angles.
     */
    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), this->getMaskImage(), std::forward<Ts>(args)...);
        calcImageDifferences(rotater);
        const size_t numSnapshots = this->getNumSnapshots();

        // Now get the minimum for each snapshot and the column this corresponds to
        std::vector<size_t> bestColumns;
        bestColumns.reserve(numSnapshots);
        std::vector<float> minDifferences;
        minDifferences.reserve(numSnapshots);
        for (size_t i = 0; i < numSnapshots; i++) {
            const auto elem = std::min_element(std::cbegin(m_RotatedDifferences[i]), std::cend(m_RotatedDifferences[i]));
            bestColumns.push_back(std::distance(std::cbegin(m_RotatedDifferences[i]), elem));
            minDifferences.push_back(*elem);
        }

        // Return result
        return std::tuple_cat(RIDFProcessor()(bestColumns, minDifferences, rotater),
                              std::make_tuple(std::cref(m_RotatedDifferences)));
    }

private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<class RotaterType>
    void calcImageDifferences(RotaterType &rotater) const
    {
        const size_t numSnapshots = this->getNumSnapshots();
        BOB_ASSERT(numSnapshots > 0);

        // Preallocate snapshot difference vectors
        while (m_RotatedDifferences.size() < numSnapshots) {
            m_RotatedDifferences.emplace_back(rotater.numRotations());
        }

        // Scan across image columns
        rotater.rotate(
                [this, numSnapshots](const cv::Mat &fr, const cv::Mat &mask, size_t i) {
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
