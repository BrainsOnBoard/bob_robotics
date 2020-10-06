#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "differencers.h"
#include "insilico_rotater.h"
#include "perfect_memory_store_raw.h"
#include "visual_navigation_base.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

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
// BoBRobotics::Navigation::PerfectMemory
//------------------------------------------------------------------------
template<typename Store = PerfectMemoryStore::RawImage<>>
class PerfectMemory : public VisualNavigationBase
{
public:
    template<class... Ts>
    PerfectMemory(const cv::Size &unwrapRes, Ts &&... args)
      : VisualNavigationBase(unwrapRes)
      , m_Store(unwrapRes, std::forward<Ts>(args)...)
    {}

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);

        // Add snapshot
        m_Store.addSnapshot(image);
    }

    virtual float test(const cv::Mat &image) const override
    {
        const auto &diffs = testInternal(image);

        // Return smallest difference
        return *std::min_element(diffs.begin(), diffs.end());
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
    const cv::Mat &getSnapshot(size_t index) const{ return m_Store.getSnapshot(index); }

    /*!
     * \brief Get differences between current view and stored snapshots
     */
    const std::vector<float> &getImageDifferences(const cv::Mat &image) const
    {
        return testInternal(image);
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

    auto &testInternal(const cv::Mat &image) const
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);

        const size_t numSnapshots = getNumSnapshots();
        BOB_ASSERT(numSnapshots > 0);

        // Clear differences
        static std::vector<float> diffs;
        #pragma omp threadprivate(diffs)
        diffs.reserve(numSnapshots);
        diffs.clear();

        // Loop through snapshots and caculate differences
        for (size_t s = 0; s < numSnapshots; s++) {
            diffs.push_back(calcSnapshotDifference(image, getMaskImage(), s));
        }

        return diffs;
    }
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryRotater
//------------------------------------------------------------------------
template<typename Store = PerfectMemoryStore::RawImage<>, typename RIDFProcessor = BestMatchingSnapshot, typename Rotater = InSilicoRotater>
class PerfectMemoryRotater : public PerfectMemory<Store>
{
public:
    template<class... Ts>
    PerfectMemoryRotater(const cv::Size unwrapRes, Ts &&... args)
      : PerfectMemory<Store>(unwrapRes, std::forward<Ts>(args)...)
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
    const auto &getImageDifferences(Ts &&... args) const
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
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
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
            size_t minIndex;
            auto minVal = m_RotatedDifferences.row(i).minCoeff(&minIndex);
            bestColumns.push_back(minIndex);
            minDifferences.push_back(minVal);
        }

        // Return result
        return std::tuple_cat(RIDFProcessor()(bestColumns, minDifferences, rotater),
                              std::make_tuple(&m_RotatedDifferences));
    }

private:
    mutable Eigen::MatrixXf m_RotatedDifferences;

    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<class RotaterType>
    void calcImageDifferences(RotaterType &rotater) const
    {
        const auto numSnapshots = this->getNumSnapshots();
        BOB_ASSERT(numSnapshots > 0);

        // Preallocate snapshot difference vectors
        m_RotatedDifferences.resize(numSnapshots, rotater.numRotations());

        // Scan across image columns
        rotater.rotate(
                [this, numSnapshots](const cv::Mat &fr, const cv::Mat &mask, size_t i) {
                    // Loop through snapshots
                    for (size_t s = 0; s < numSnapshots; s++) {
                        // Calculate difference
                        m_RotatedDifferences(s, i) = this->calcSnapshotDifference(fr, mask, s);
                    }
                });
    }
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics
