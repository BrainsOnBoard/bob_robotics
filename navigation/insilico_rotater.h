#pragma once

// BoB robotics includes
#include "../common/assert.h"

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::InSilicoRotater
//------------------------------------------------------------------------
/*!
 * \brief A "rotater" for PerfectMemoryBase classes which rotates a panoramic
 *        image by moving columns of pixels from one side of the image to the
 *        other
 */
class InSilicoRotater
{
public:
    InSilicoRotater(const cv::Size &unwrapRes,
                    const cv::Mat &maskImage,
                    const cv::Mat &image,
                    size_t scanStep = 1,
                    size_t beginRoll = 0,
                    size_t endRoll = std::numeric_limits<size_t>::max())
    :   m_ScanStep(scanStep), m_BeginRoll(beginRoll), 
        m_EndRoll((endRoll == std::numeric_limits<size_t>::max()) ? static_cast<size_t>(m_Image.cols) : endRoll)
    {
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);
        BOB_ASSERT(image.type() == CV_8UC1);

        image.copyTo(m_Image);
        maskImage.copyTo(m_MaskImage);
    }

    template<class Func>
    void rotate(Func func)
    {
        if(m_BeginRoll != 0) {
            rollImage(m_Image, m_BeginRoll);
            if (!m_MaskImage.empty()) {
                rollImage(m_MaskImage, m_BeginRoll);
            }
        }
        
        size_t i = m_BeginRoll;
        while (true) {
            func(m_Image, m_MaskImage, i - m_BeginRoll);

            i += m_ScanStep;
            if (i >= (size_t)m_EndRoll) {
                break;
            }

            rollImage(m_Image, m_ScanStep);
            if (!m_MaskImage.empty()) {
                rollImage(m_MaskImage, m_ScanStep);
            }
        }
    }

    size_t max() const
    {
        return (m_EndRoll - m_BeginRoll) / m_ScanStep;
    }

private:
    const size_t m_ScanStep;
    const size_t m_BeginRoll;
    const size_t m_EndRoll;
    cv::Mat m_Image, m_MaskImage;

    static void rollImage(cv::Mat &image, size_t pixels)
    {
        // Loop through rows
        for (int y = 0; y < image.rows; y++) {
            // Get pointer to start of row
            uint8_t *rowPtr = image.ptr(y);

            // Rotate row to left by m_ScanStep pixels
            std::rotate(rowPtr, rowPtr + pixels, rowPtr + image.cols);
        }
    }
};
} // Navigation
} // BoBRobotics
