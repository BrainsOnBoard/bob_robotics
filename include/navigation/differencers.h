/*
 * This header contains small, functor-type classes for calculating the
 * differences between images, used by PerfectMemory.
 */

#pragma once

// BoB robotics includes
#include "common/macros.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

// Standard C includes
#include <cmath>
#include <cstddef>

namespace BoBRobotics {
namespace Navigation {

template<class Derived, class VecType>
class DifferencerBase
{
public:
    float operator()(cv::InputArray &src1,
                     cv::InputArray &src2,
                     const cv::Mat &mask1 = {},
                     const cv::Mat &mask2 = {})
    {
        auto obj = static_cast<Derived *>(this);
        size_t count = obj->calculate(src1, src2, m_ScratchVector, mask1, mask2);
        return obj->mean(m_ScratchVector, count, mask1, mask2);
    }

    VecType &getScratchVector() { return m_ScratchVector; }

protected:
    cv::Mat &combineMasks(const cv::Mat &mask1, const cv::Mat &mask2)
    {
        BOB_ASSERT(mask1.empty() || mask1.type() == CV_8UC1);
        BOB_ASSERT(mask2.empty() || mask2.type() == CV_8UC1);

        if (mask1.empty()) {
            m_MaskScratch = mask2;
        } else if (mask2.empty()) {
            m_MaskScratch = mask1;
        } else {
#ifdef DEBUG
            // Only do these extra checks if we're debugging because they're costly
            const auto check = [](const cv::Mat &mask) {
                BOB_ASSERT(std::all_of(mask.datastart, mask.dataend, [](uint8_t val) {
                    return val == 0 || val == 0xff;
                }));
            };
            check(mask1);
            check(mask2);
#endif

            cv::bitwise_and(mask1, mask2, m_MaskScratch);
        }

        return m_MaskScratch;
    }

private:
    cv::Mat m_MaskScratch;
    VecType m_ScratchVector;
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::AbsDiff
//------------------------------------------------------------------------
/*!
 * \brief For calculating the mean absolute difference between images
 *
 * Can be passed to PerfectMemory as a template parameter.
 */
class AbsDiff
{
public:
    template<class VecType = cv::Mat>
    class Internal
      : public DifferencerBase<AbsDiff::Internal<VecType>, VecType>
    {
    public:
        size_t calculate(cv::InputArray &src1, cv::InputArray &src2,
                         cv::OutputArray &dst, const cv::Mat &, const cv::Mat &)
        {
            cv::absdiff(src1, src2, dst);
            return 0;
        }

        float mean(cv::InputArray &arr, size_t, const cv::Mat &mask1,
                   const cv::Mat &mask2)
        {
            return cv::mean(arr, this->combineMasks(mask1, mask2))[0];
        }
    };
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::RMSDiff
//------------------------------------------------------------------------
/*!
 * \brief For calculating the root mean square difference between images
 *
 * Can be passed to PerfectMemory as a template parameter.
 */
class RMSDiff
{
public:
    template<class VecType = cv::Mat>
    class Internal
      : public DifferencerBase<RMSDiff::Internal<VecType>, VecType>
    {
    public:
        size_t calculate(cv::InputArray &src1, cv::InputArray &src2,
                         cv::OutputArray &dst, const cv::Mat &mask1,
                         const cv::Mat &mask2)
        {
            // Get pixel-wise absolute difference
            cv::absdiff(src1, src2, dst);
            BOB_ASSERT(dst.type() == CV_8UC1);
            auto dstMat = dst.getMat();

            const auto &mask = this->combineMasks(mask1, mask2);
            size_t n;
            if (mask.empty()) {
                auto size = src1.size();
                n = size.width * size.height;
            } else {
                // If there's a mask, we should zero out the unused pixels
                cv::bitwise_and(dstMat, mask, dstMat);
                n = static_cast<size_t>(cv::sum(mask)[0]) / 0xff;
            }

            // Square the differences
            const auto sz = dstMat.size();
            m_Differences.resize(sz.width * sz.height);
            const auto sq = [](uint8_t val) {
                float fval = val;
                return fval * fval;
            };
            std::transform(dstMat.datastart, dstMat.dataend, m_Differences.begin(), sq);

            return n;
        }

        float mean(cv::InputArray &, size_t count, const cv::Mat &,
                   const cv::Mat &)
        {
            return sqrtf(cv::sum(m_Differences)[0] / (float) count);
        }

    private:
        // We need a second scratch variable to store square differences
        std::vector<float> m_Differences;
    };
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::CorrCoefficient
//------------------------------------------------------------------------
/*!
 * \brief For calculating difference based on Pearson's correlation coefficient
 *
 * Can be passed to PerfectMemory as a template parameter.
 *
 * Note that we use 1 - abs(Pearson's rho) to give us a difference value, as we
 * want to calculate the dissimilarity between images.
 */
struct CorrCoefficient {
    /*
     * NB: This template parameter is unused, but left in for consistency with
     * other differencer classes.
     */
    template<class VecType = void>
    class Internal
    {
    public:
        float operator()(cv::InputArray &src1, cv::InputArray &src2,
                         const cv::Mat &mask1 = {}, const cv::Mat &mask2 = {})
        {
            // Don't support masks for now
            BOB_ASSERT(mask1.empty() && mask2.empty());

            const auto allSame = [](cv::InputArray &arr) {
                const auto m = arr.getMat();
                BOB_ASSERT(m.type() == CV_8UC1);
                auto beg = m.datastart;
                auto end = m.dataend;
                return std::all_of(beg + 1, end,
                                   [beg](uint8_t val) { return val == *beg; });
            };
            if (allSame(src1) || allSame(src2)) {
                throw std::invalid_argument("Vectors src1 and src2 must have "
                                            "more than one unique value (e.g. "
                                            "they cannot be all zeros)");
            }

            std::array<float, 1> dst;
            cv::matchTemplate(src1, src2, dst, cv::TM_CCOEFF_NORMED);

            /*
             * As we're interested in *dissimilarity* between images, we use
             * 1 - correlation coefficient. For our purposes, it doesn't matter
             * whether the correlation is positive or negative.
             */
            return 1.f - fabsf(dst[0]);
        }
    };
};

} // Navigation
} // BoBRobotics
