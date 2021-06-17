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

// TBB
#include <tbb/parallel_for.h>

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
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);

        // Add snapshot
        m_Store.addSnapshot(image, mask);
    }

    virtual float test(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) const override
    {
        return test(getFullWindow(), image);
    }

    virtual void clearMemory() override
    {
        m_Store.clear();
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    //! Test the algorithm with the specified image within a specified 'window' of snapshots
    float test(const Window &window, const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) const
    {
        testInternal(window, image, mask);

        // Return smallest difference
        return *std::min_element(m_Differences.begin(), m_Differences.end());
    }

    //! Return the number of snapshots that have been read into memory
    size_t getNumSnapshots() const{ return m_Store.getNumSnapshots(); }

    //! Return a specific snapshot
    const cv::Mat &getSnapshot(size_t index) const{ return m_Store.getSnapshot(index).first; }

    //! Return a specific snapshot and mask associated with it
    const std::pair<cv::Mat, ImgProc::Mask> &getMaskedSnapshot(size_t index) const{ return m_Store.getSnapshot(index); }

    //! Get differences between current view and all stored snapshots
    const std::vector<float> &getImageDifferences(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) const
    {
        testInternal(getFullWindow(), image, mask);
        return m_Differences;
    }

    //! Get differences between current view and specified 'window' of stored snapshots
    const std::vector<float> &getImageDifferences(const Window &window, const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) const
    {
        testInternal(window, image, mask);
        return m_Differences;
    }

    Window getFullWindow() const
    {
        return {0, getNumSnapshots()};
    }

protected:
    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    float calcSnapshotDifference(const cv::Mat &image,
                                 const ImgProc::Mask &mask, size_t snapshot) const
    {
        return m_Store.calcSnapshotDifference(image, mask, snapshot);
    }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    Store m_Store;
    mutable std::vector<float> m_Differences;

    void testInternal(const Window &window, const cv::Mat &image, const ImgProc::Mask &mask) const
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);
        if(!mask.empty()) {
            BOB_ASSERT(mask.get().cols == unwrapRes.width);
            BOB_ASSERT(mask.get().rows == unwrapRes.height);
            BOB_ASSERT(mask.get().type() == CV_8UC1);
        }

        BOB_ASSERT(window.first < getNumSnapshots());
        BOB_ASSERT(window.second <= getNumSnapshots());
        BOB_ASSERT(window.first < window.second);

        m_Differences.resize(window.second - window.first);

        // Loop through snapshots and calculate differences
        tbb::parallel_for(tbb::blocked_range<size_t>(window.first, window.second),
            [&](const auto &r) {
                for (size_t s = r.begin(); s != r.end(); ++s) {
                    m_Differences[s - window.first] = calcSnapshotDifference(image, mask, s);
                }
            });
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
    const auto &getImageDifferences(typename PerfectMemory<Store>::Window window, ImgProc::Mask mask, Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), mask, std::forward<Ts>(args)...);
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
    const auto &getImageDifferences(typename PerfectMemory<Store>::Window window, Ts &&... args) const
    {
        return getImageDifferences(window, ImgProc::Mask{}, std::forward<Ts>(args)...);
    }

    /*!
     * \brief Get differences between current view and stored snapshots
     *
     * The parameters are perfect-forwarded to the Rotater class, so e.g. for
     * InSilicoRotater one passes in a cv::Mat and (optionally) an unsigned int
     * for the scan step.
     */
    template<class... Ts>
    const auto &getImageDifferences(ImgProc::Mask mask, Ts &&... args) const
    {
        return getImageDifferences(getFullWindow(), mask, std::forward<Ts>(args)...);
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
        return getImageDifferences(getFullWindow(), ImgProc::Mask{}, std::forward<Ts>(args)...);
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
    auto getHeading(typename PerfectMemory<Store>::Window window, ImgProc::Mask mask, Ts &&... args) const
    {
        auto rotater = Rotater::create(this->getUnwrapResolution(), mask, std::forward<Ts>(args)...);
        calcImageDifferences(window, rotater);

        // Now get the minimum for each snapshot and the column this corresponds to
        const size_t numSnapshots = window.second - window.first;
        m_BestColumns.resize(numSnapshots);
        m_MinimumDifferences.resize(numSnapshots);

        tbb::parallel_for(tbb::blocked_range<size_t>(0, numSnapshots),
                          [&](const auto &r) {
                              for (size_t i = r.begin(); i != r.end(); ++i) {
                                  m_MinimumDifferences[i] = m_RotatedDifferences.row(i).minCoeff(&m_BestColumns[i]);
                              }
                          });

        // Return result
        return std::tuple_cat(RIDFProcessor()(m_BestColumns, m_MinimumDifferences, rotater, window.first),
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
    auto getHeading(ImgProc::Mask mask, Ts &&... args) const
    {
        return getHeading(this->getFullWindow(), mask, std::forward<Ts>(args)...);
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
    auto getHeading(typename PerfectMemory<Store>::Window window, Ts &&... args) const
    {
        return getHeading(window, ImgProc::Mask{}, std::forward<Ts>(args)...);
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
        return getHeading(this->getFullWindow(), ImgProc::Mask{}, std::forward<Ts>(args)...);
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
        BOB_ASSERT(window.first < this->getNumSnapshots());
        BOB_ASSERT(window.second <= this->getNumSnapshots());
        BOB_ASSERT(window.first < window.second);

        // Preallocate snapshot difference vectors
        m_RotatedDifferences.resize(window.second - window.first, rotater.numRotations());

        // Scan across image columns
        rotater.rotate(
                [this, &window](const cv::Mat &fr, const ImgProc::Mask &mask, size_t i) {
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
