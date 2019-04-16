#pragma once

// BoB robotics includes
#include "common/assert.h"

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
                        const cv::Mat &maskImage,
                        const cv::Mat &image,
                        size_t scanStep,
                        IterType beginRoll,
                        IterType endRoll)
          : m_ScanStep(scanStep)
          , m_BeginRoll(beginRoll)
          , m_EndRoll(endRoll)
          , m_ImageOriginal(image)
          , m_MaskImageOriginal(maskImage)
          , m_Image(unwrapRes.height, unwrapRes.width, CV_8UC1)
          , m_MaskImage(maskImage.rows, maskImage.cols, maskImage.type())
        {
            BOB_ASSERT(image.cols == unwrapRes.width);
            BOB_ASSERT(image.rows == unwrapRes.height);
            BOB_ASSERT(image.type() == CV_8UC1);
            BOB_ASSERT(image.isContinuous());
            BOB_ASSERT(beginRoll < endRoll);
            BOB_ASSERT((distance(endRoll, beginRoll) % scanStep) == 0);

            const auto index = toIndex(beginRoll);
            rollImage(image, m_Image, index);
            if (!maskImage.empty()) {
                rollImage(maskImage, m_MaskImage, index);
            }
        }

        template<class Func>
        void rotate(Func func)
        {
            auto i = m_BeginRoll;
            while (true) {
                func(m_Image, m_MaskImage, distance(m_BeginRoll, i));

                i += m_ScanStep;
                if (i >= m_EndRoll) {
                    break;
                }

                const auto index = toIndex(i);
                rollImage(m_ImageOriginal, m_Image, index);
                if (!m_MaskImageOriginal.empty()) {
                    rollImage(m_MaskImageOriginal, m_MaskImage, index);
                }
            }
        }

        units::angle::radian_t columnToHeading(size_t column) const
        {
            return units::angle::turn_t{ (double) toIndex(column) / (double) m_ImageOriginal.cols };
        }

        size_t numRotations() const
        {
            return (m_EndRoll - m_BeginRoll) / m_ScanStep;
        }

    private:
        const size_t m_ScanStep;
        const IterType m_BeginRoll, m_EndRoll;
        const cv::Mat &m_ImageOriginal, &m_MaskImageOriginal;
        cv::Mat m_Image, m_MaskImage;

        static void rollImage(const cv::Mat &imageIn, cv::Mat &imageOut, size_t pixels)
        {
            // Loop through rows
            for (int y = 0; y < imageIn.rows; y++) {
                // Get pointer to start of row
                const uint8_t *rowPtr = imageIn.ptr(y);
                uint8_t *rowPtrOut = imageOut.ptr(y);

                // Rotate row to left by pixels
                std::rotate_copy(rowPtr, rowPtr + pixels, rowPtr + imageIn.cols, rowPtrOut);
            }
        }

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
           const cv::Mat &maskImage,
           const cv::Mat &image,
           IterType beginRoll,
           IterType endRoll)
    {
        return RotaterInternal<IterType>(unwrapRes, maskImage, image, 1, beginRoll, endRoll);
    }

    static auto
    create(const cv::Size &unwrapRes,
           const cv::Mat &maskImage,
           const cv::Mat &image,
           size_t scanStep,
           size_t beginRoll,
           size_t endRoll)
    {
        return RotaterInternal<size_t>(unwrapRes, maskImage, image, scanStep, beginRoll, endRoll);
    }

    static auto
    create(const cv::Size &unwrapRes,
           const cv::Mat &maskImage,
           const cv::Mat &image,
           size_t scanStep = 1,
           size_t beginRoll = 0)
    {
        return RotaterInternal<size_t>(unwrapRes, maskImage, image, scanStep, beginRoll, image.cols);
    }
};
} // Navigation
} // BoBRobotics
