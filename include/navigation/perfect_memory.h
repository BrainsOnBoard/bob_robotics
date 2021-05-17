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
#include <utility>
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
    // Typedefines
    //------------------------------------------------------------------------
    typedef std::pair<size_t, size_t> Window;

    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    static const Window FullWindow;

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
        return test(FullWindow, image);
    }

    virtual void clearMemory() override
    {
        m_Store.clear();
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    //! Test the algorithm with the specified image within a specified 'window' of snapshots
    float test(const Window &window, const cv::Mat &image) const
    {
        testInternal(window, image);

        // Return smallest difference
        return *std::min_element(m_Differences.begin(), m_Differences.end());
    }

    //! Return the number of snapshots that have been read into memory
    size_t getNumSnapshots() const{ return m_Store.getNumSnapshots(); }

    //! Return a specific snapshot
    const cv::Mat &getSnapshot(size_t index) const{ return m_Store.getSnapshot(index); }

    //! Get differences between current view and all stored snapshots
    const std::vector<float> &getImageDifferences(const cv::Mat &image) const
    {
        testInternal(FullWindow, image);
        return m_Differences;
    }

    //! Get differences between current view and specified 'window' of stored snapshots
    const std::vector<float> &getImageDifferences(const Window &window, const cv::Mat &image) const
    {
        testInternal(window, image);
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

    void testInternal(const Window &window, const cv::Mat &image) const
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);
        BOB_ASSERT(window.first < getNumSnapshots());
        BOB_ASSERT(window.second < getNumSnapshots());
        BOB_ASSERT(window.first < window.second);

        m_Differences.resize(window.second - window.first);

        // Loop through snapshots and caculate differences
        #pragma omp parallel for
        for (size_t s = window.first; s < window.second; s++) {
            m_Differences[s - window.first] = calcSnapshotDifference(image, getMaskImage(), s);
        }
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
     * \brief Get differences between current view and stored snapshots within a 'window'
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step. **NOTE** I wanted window to be a const reference but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    const auto &getImageDifferences(typename PerfectMemory<Store>::Window window, Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), this->getMaskImage(), std::forward<Ts>(args)...);
        calcImageDifferences(window, rotater);
        return m_RotatedDifferences;
    }

    /*!
     * \brief Get differences between current view and stored snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step.
     */
    template<class... Ts>
    const auto &getImageDifferences(Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), this->getMaskImage(), std::forward<Ts>(args)...);
        calcImageDifferences({0, getNumSnapshots()}, rotater);
        return m_RotatedDifferences;
    }

    /*!
     * \brief Get an estimate for heading based on comparing image with stored
     *        snapshots within a 'window'
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step. **NOTE** I wanted window to be a const reference but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    auto getHeading(typename PerfectMemory<Store>::Window window, Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), this->getMaskImage(), std::forward<Ts>(args)...);
        calcImageDifferences(window, rotater);

        // Now get the minimum for each snapshot and the column this corresponds to
        const size_t numSnapshots = window.second - window.first;
        m_BestColumns.resize(numSnapshots);
        m_MinimumDifferences.resize(numSnapshots);

        #pragma omp parallel for
        for (size_t i  = 0; i < numSnapshots; i++) {
            m_MinimumDifferences[i] = m_RotatedDifferences.row(i).minCoeff(&m_BestColumns[i]);
        }

        // Return result
        return std::tuple_cat(RIDFProcessor()(m_BestColumns, m_MinimumDifferences, rotater),
                              std::make_tuple(&m_RotatedDifferences));
    }

    /*!
     * \brief Get an estimate for heading based on comparing image with stored
     *        snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step.
     */
    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        return getHeading({0, getNumSnapshots()}, std::forward<Ts>(args)...);
    }

private:
    mutable Eigen::MatrixXf m_RotatedDifferences;
    mutable std::vector<size_t> m_BestColumns;
    mutable std::vector<float> m_MinimumDifferences;

    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<class RotaterType>
    void calcImageDifferences(typename PerfectMemory<Store>::Window window, RotaterType &rotater) const
    {
        BOB_ASSERT(window.first < getNumSnapshots());
        BOB_ASSERT(window.second < getNumSnapshots());
        BOB_ASSERT(window.first < window.second);

        // Preallocate snapshot difference vectors
        m_RotatedDifferences.resize(window.second - window.first, rotater.numRotations());

        // Scan across image columns
        rotater.rotate(
                [this, &window](const cv::Mat &fr, const cv::Mat &mask, size_t i) {
                    // Loop through snapshots
                    for (size_t s = window.first; s < window.second; s++) {
                        // Calculate difference
                        m_RotatedDifferences(s - window.first, i) = this->calcSnapshotDifference(fr, mask, s);
                    }
                });
    }
};
} // Navigation
} // BoBRobotics
