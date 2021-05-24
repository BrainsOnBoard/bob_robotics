#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "imgproc/mask.h"
#include "imgproc/roll.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InSilicoRotater
//------------------------------------------------------------------------
/*!
 * \brief A "rotater" for PerfectMemoryBase classes which rotates a panoramic
 *        image by moving columns of pixels from one side of the image to the
 *        other
 */
struct InSilicoRotater
{
    template<typename IterType>
    class RotaterInternal
    {
    public:
        RotaterInternal(const cv::Size &unwrapRes,
                        const ImgProc::Mask &mask,
                        const cv::Mat &image,
                        size_t scanStep,
                        IterType beginRoll,
                        IterType endRoll)
          : m_ScanStep(scanStep)
          , m_BeginRoll(beginRoll)
          , m_EndRoll(endRoll)
          , m_Image(image)
          , m_Mask(mask)
        {
            BOB_ASSERT(image.cols == unwrapRes.width);
            BOB_ASSERT(image.rows == unwrapRes.height);
            BOB_ASSERT(image.type() == CV_8UC1);
            BOB_ASSERT(image.isContinuous());
            BOB_ASSERT(beginRoll < endRoll);
            BOB_ASSERT((distance(endRoll, beginRoll) % scanStep) == 0);
      }

        template<class Func>
        void rotate(Func func) const
        {
            #pragma omp parallel
            {
                static cv::Mat scratchImage;
                static ImgProc::Mask scratchMask;
                #pragma omp threadprivate(scratchImage, scratchMask)

                #pragma omp for
                for (auto i = m_BeginRoll; i < m_EndRoll; i += m_ScanStep) {
                    const auto index = toIndex(i);
                    ImgProc::roll(m_Image, scratchImage, index);
                    m_Mask.roll(scratchMask, index);

                    func(scratchImage, scratchMask, distance(m_BeginRoll, i));
                }
            }
       }

        units::angle::radian_t columnToHeading(size_t column) const
        {
            return units::angle::turn_t{ (double) toIndex(m_BeginRoll + column) / (double) m_Image.cols };
        }

        size_t numRotations() const
        {
            return (m_EndRoll - m_BeginRoll) / m_ScanStep;
        }

    private:
        const size_t m_ScanStep;
        const IterType m_BeginRoll, m_EndRoll;
        const cv::Mat &m_Image;
        const ImgProc::Mask m_Mask;

        static size_t distance(size_t first, size_t last)
        {
            return last - first;
        }

        template<typename Iter>
        static size_t distance(Iter first, Iter last)
        {
            return static_cast<size_t>(std::distance(first, last));
        }

        static size_t toIndex(size_t index)
        {
            return index;
        }

        template<typename Iter>
        static size_t toIndex(Iter it)
        {
            return *it;
        }
    };

    template<typename IterType>
    static auto
    create(const cv::Size &unwrapRes,
           const ImgProc::Mask &mask,
           const cv::Mat &image,
           IterType beginRoll,
           IterType endRoll)
    {
        return RotaterInternal<IterType>(unwrapRes, mask, image, 1, beginRoll, endRoll);
    }

    static auto
    create(const cv::Size &unwrapRes,
           const ImgProc::Mask &mask,
           const cv::Mat &image,
           size_t scanStep,
           size_t beginRoll,
           size_t endRoll)
    {
        return RotaterInternal<size_t>(unwrapRes, mask, image, scanStep, beginRoll, endRoll);
    }

    static auto
    create(const cv::Size &unwrapRes,
           const ImgProc::Mask &mask,
           const cv::Mat &image,
           size_t scanStep = 1,
           size_t beginRoll = 0)
    {
        return RotaterInternal<size_t>(unwrapRes, mask, image, scanStep, beginRoll, image.cols);
    }
};
} // Navigation
} // BoBRobotics
