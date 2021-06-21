#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "differencers.h"
#include "insilico_rotater.h"
#include "perfect_memory_store_raw.h"

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
class PerfectMemory
{
public:
    template<class... Ts>
    PerfectMemory(const cv::Size &unwrapRes, Ts &&... args)
      : m_UnwrapRes(unwrapRes)
      , m_Store(unwrapRes, std::forward<Ts>(args)...)
    {}

    //------------------------------------------------------------------------
    // Typedefines
    //------------------------------------------------------------------------
    typedef std::pair<size_t, size_t> Window;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void train(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{})
    {
        const auto &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);

        // Add snapshot
        m_Store.addSnapshot(image, mask);
    }

    float test(const cv::Mat &image, ImgProc::Mask &mask = ImgProc::Mask{},
               const Window &window = getFullWindow()) const
    {
        testInternal(image, mask, window);

        // Return smallest difference
        return *std::min_element(m_Differences.begin(), m_Differences.end());
    }

    void clearMemory()
    {
        m_Store.clear();
    }

    //! Return the number of snapshots that have been read into memory
    size_t getNumSnapshots() const{ return m_Store.getNumSnapshots(); }

    //! Return a specific snapshot
    const cv::Mat &getSnapshot(size_t index) const{ return m_Store.getSnapshot(index).first; }

    //! Return a specific snapshot and mask associated with it
    const std::pair<cv::Mat, ImgProc::Mask> &getMaskedSnapshot(size_t index) const{ return m_Store.getSnapshot(index); }

    //! Get differences between current view and all stored snapshots
    const std::vector<float> &getImageDifferences(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{},
                                                  const Window &window = getFullWindow()) const
    {
        testInternal(image, mask, window);
        return m_Differences;
    }

    Window getFullWindow() const
    {
        return {0, getNumSnapshots()};
    }

    //! Get the resolution of images
    const cv::Size &getUnwrapResolution() const { return m_UnwrapRes; }

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
    const cv::Size m_UnwrapRes;
    Store m_Store;
    mutable std::vector<float> m_Differences;

    void testInternal(const cv::Mat &image, const ImgProc::Mask &mask, const Window &window) const
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
template<typename Store = PerfectMemoryStore::RawImage<>, typename RIDFProcessor = BestMatchingSnapshot>
class PerfectMemoryRotater : public PerfectMemory<Store>
{
public:
    template<class... Ts>
    PerfectMemoryRotater(const cv::Size unwrapRes, Ts &&... args)
      : PerfectMemory<Store>(unwrapRes, std::forward<Ts>(args)...)
    {
    }

    /*!
     * \brief Get differences between current view with mask and stored snapshots within a 'window'
     *
     * Any additional parameters specifying rotation constraints are perfect-forwarded to 
     * InSilicoRotater::create. **NOTE** I wanted mask and window to be const references but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    const auto &getImageDifferences(const cv::Mat &image, ImgProc::Mask mask, Window window, Ts &&... args) const
    {
        auto rotater = InSilicoRotater::create(this->getUnwrapResolution(), mask, image, std::forward<Ts>(args)...);
        calcImageDifferences(window, rotater);
        return m_RotatedDifferences;
    }

    /*!
     * \brief Get differences between current view with mask and stored snapshots
     *
     * Any additional parameters specifying rotation constraints are perfect-forwarded to 
     * InSilicoRotater::create. **NOTE**I wanted mask and window to be const references but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    const auto &getImageDifferences(const cv::Mat &image, ImgProc::Mask mask, Ts &&... args) const
    {
        return getImageDifferences(image, mask, getFullWindow(), std::forward<Ts>(args)...);
    }

    /*!
     * \brief Get differences between current view and stored snapshots
     *
     * Any additional parameters specifying rotation constraints are perfect-forwarded to 
     * InSilicoRotater::create. **NOTE** I wanted mask and window to be const references but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    const auto &getImageDifferences(const cv::Mat &image, Ts &&... args) const
    {
        return getImageDifferences(image, ImgProc::Mask{}, getFullWindow(), std::forward<Ts>(args)...);
    }

    /*!
     * \brief Get an estimate for heading based on current view with mask
     *        and stored snapshots within a 'window'
     *
     * Any additional parameters specifying rotation constraints are perfect-forwarded to 
     * InSilicoRotater::create. **NOTE**I wanted mask and window to be const references but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    auto getHeading(const cv::Mat &image, ImgProc::Mask mask, Window window, Ts &&... args) const
    {
        auto rotater = InSilicoRotater::create(this->getUnwrapResolution(), mask, image, std::forward<Ts>(args)...);
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
     * \brief Get an estimate for heading based on current view with mask and stored snapshots 
     *
     * Any additional parameters specifying rotation constraints are perfect-forwarded to 
     * InSilicoRotater::create. **NOTE**I wanted mask and window to be const references but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    auto getHeading(const cv::Mat &image, ImgProc::Mask mask, Ts &&... args) const
    {
        return getHeading(image, mask, this_>getFullWindow(), std::forward<Ts>(args)...);
    }

    /*!
     * \brief Get an estimate for heading based on current view  and stored snapshots
     *
     * Any additional parameters specifying rotation constraints are perfect-forwarded to 
     * InSilicoRotater::create. **NOTE**I wanted mask and window to be const references but for
     * reasons that are beyond me, if it is a reference the second overload always gets selected
     */
    template<class... Ts>
    auto getHeading(const cv::Mat &image, Ts &&... args) const
    {
        return getHeading(image, ImgProc::Mask{}, this->getFullWindow(), std::forward<Ts>(args)...);
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
