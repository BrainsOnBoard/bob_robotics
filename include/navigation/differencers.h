/*
 * This header contains small, functor-type classes for calculating the
 * differences between images, used by PerfectMemory.
 */

#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "imgproc/mask.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <numeric>
#include <stdexcept>
#include <tuple>
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
                     const ImgProc::Mask &mask1 = {},
                     const ImgProc::Mask &mask2 = {})
    {
        auto obj = static_cast<Derived *>(this);
        size_t count = obj->calculate(src1, src2, m_ScratchVector, mask1, mask2);
        return obj->mean(m_ScratchVector, count, mask1, mask2);
    }

    VecType &getScratchVector() { return m_ScratchVector; }

private:
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
                         cv::OutputArray &dst, const ImgProc::Mask &,
                         const ImgProc::Mask &)
        {
            cv::absdiff(src1, src2, dst);
            return 0;
        }

        float mean(cv::InputArray &arr, size_t, const ImgProc::Mask &mask1,
                   const ImgProc::Mask &mask2)
        {
            mask1.combine(mask2, m_CombinedMask);
            return cv::mean(arr, m_CombinedMask.get())[0];
        }

    private:
        ImgProc::Mask m_CombinedMask;
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
                         cv::OutputArray &dst, const ImgProc::Mask &mask1,
                         const ImgProc::Mask &mask2)
        {
            // Get pixel-wise absolute difference
            cv::absdiff(src1, src2, dst);
            BOB_ASSERT(dst.type() == CV_8UC1);
            auto dstMat = dst.getMat();

            mask1.combine(mask2, m_CombinedMask);
            m_CombinedMask.apply(dstMat, dstMat);

            // Square the differences
            const auto sz = dstMat.size();
            m_Differences.resize(sz.width * sz.height);
            const auto sq = [](uint8_t val) {
                float fval = val;
                return fval * fval;
            };
            std::transform(dstMat.datastart, dstMat.dataend, m_Differences.begin(), sq);

            return m_CombinedMask.countUnmaskedPixels(src1.size());
        }

        float mean(cv::InputArray &, size_t count, const ImgProc::Mask &,
                   const ImgProc::Mask &)
        {
            return sqrtf(cv::sum(m_Differences)[0] / (float) count);
        }

    private:
        // We need a second scratch variable to store square differences
        std::vector<float> m_Differences;
        ImgProc::Mask m_CombinedMask;
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
     * NB: We don't need any scratch storage, but the user can choose to have
     * some (e.g. for the HOG flavour of perfect memory).
     */
    template<class VecType = std::tuple<>>
    class Internal
      : public DifferencerBase<CorrCoefficient::Internal<VecType>, VecType>
    {
    public:
        float operator()(cv::InputArray &src1, cv::InputArray &src2,
                         const ImgProc::Mask &mask1 = {},
                         const ImgProc::Mask &mask2 = {})
        {
            mask1.combine(mask2, m_CombinedMask);

            const float std1 = centreAndGetStdDev(src1, m_CombinedMask, m_Scratch1);
            const float std2 = centreAndGetStdDev(src2, m_CombinedMask, m_Scratch2);

            cv::multiply(m_Scratch1, m_Scratch2, m_ScratchMult);
            const float abMean = cv::mean(m_ScratchMult, m_CombinedMask.get())[0];

            const float rho = abMean / (std1 * std2);
            return 1.f - fabsf(rho);
        }

    private:
        cv::Mat m_Scratch1, m_Scratch2, m_ScratchMult;
        ImgProc::Mask m_CombinedMask;

        static float centreAndGetStdDev(cv::InputArray &src,
                                        const ImgProc::Mask &mask,
                                        cv::Mat &dst)
        {
            /*
             * Unfortunately we have to use doubles for all this as
             * cv::meanStdDev only works with double-type matrices.
             */
            src.getMat().convertTo(dst, CV_64F);
            cv::Scalar mean, std;
            cv::meanStdDev(dst, mean, std, mask.get());
            cv::subtract(dst, mean[0], dst, mask.get());
            if (std[0] == 0.0) {
                throw std::invalid_argument("Vectors src1 and src2 must have "
                                            "more than one unique value (e.g. "
                                            "they cannot be all zeros)");
            }
            return std[0];
        }
    };
};

} // Navigation
} // BoBRobotics
