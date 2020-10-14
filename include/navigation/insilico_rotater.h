#pragma once

// BoB robotics includes
#include "algorithms.h"
#include "common/macros.h"

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
    class RotaterInternal
    {
    public:
        RotaterInternal(const cv::Size &unwrapRes,
                        const cv::Mat &maskImage,
                        const cv::Mat &image,
                        size_t scanStep,
                        size_t beginRoll,
                        size_t endRoll)
          : m_ScanStep(scanStep)
          , m_BeginRoll(beginRoll)
          , m_EndRoll(endRoll)
          , m_Image(image)
          , m_MaskImage(maskImage)
        {
            BOB_ASSERT(image.cols == unwrapRes.width);
            BOB_ASSERT(image.rows == unwrapRes.height);
            BOB_ASSERT(image.type() == CV_8UC1);
            BOB_ASSERT(image.isContinuous());
            BOB_ASSERT(beginRoll < endRoll);
            BOB_ASSERT(((endRoll - beginRoll) % scanStep) == 0);
        }

        template<class Func>
        void rotate(Func func) const
        {
            forEachRotation(m_Image, func, m_MaskImage, m_ScanStep, m_BeginRoll, m_EndRoll);
        }

        units::angle::radian_t columnToHeading(size_t column) const
        {
            return units::angle::turn_t{ (double) (m_BeginRoll + column) / (double) m_Image.cols };
        }

        size_t numRotations() const
        {
            return (m_EndRoll - m_BeginRoll) / m_ScanStep;
        }

    private:
        const size_t m_ScanStep;
        const size_t m_BeginRoll, m_EndRoll;
        const cv::Mat &m_Image, &m_MaskImage;
    };

    template<typename size_t>
    static auto
    create(const cv::Size &unwrapRes,
           const cv::Mat &maskImage,
           const cv::Mat &image,
           size_t beginRoll,
           size_t endRoll)
    {
        return RotaterInternal(unwrapRes, maskImage, image, 1, beginRoll, endRoll);
    }

    static auto
    create(const cv::Size &unwrapRes,
           const cv::Mat &maskImage,
           const cv::Mat &image,
           size_t scanStep,
           size_t beginRoll,
           size_t endRoll)
    {
        return RotaterInternal(unwrapRes, maskImage, image, scanStep, beginRoll, endRoll);
    }

    static auto
    create(const cv::Size &unwrapRes,
           const cv::Mat &maskImage,
           const cv::Mat &image,
           size_t scanStep = 1,
           size_t beginRoll = 0)
    {
        return RotaterInternal(unwrapRes, maskImage, image, scanStep, beginRoll, image.cols);
    }
};
} // Navigation
} // BoBRobotics
