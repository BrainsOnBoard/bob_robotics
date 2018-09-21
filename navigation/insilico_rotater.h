#pragma once

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
                    const unsigned int scanStep = 1)
      : m_ScanStep(scanStep)
    {
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        image.copyTo(m_Image);
        maskImage.copyTo(m_MaskImage);
    }

    template<class Func>
    void operator()(Func func)
    {
        size_t i = 0;
        while (true) {
            func(m_Image, m_MaskImage, i);

            i += m_ScanStep;
            if (i >= (size_t) m_Image.cols) {
                break;
            }

            rollImage(m_Image);
            if (!m_MaskImage.empty()) {
                rollImage(m_MaskImage);
            }
        }
    }

    size_t max() const
    {
        return m_Image.cols / m_ScanStep;
    }

private:
    const unsigned int m_ScanStep;
    cv::Mat m_Image, m_MaskImage;

    void rollImage(cv::Mat &image)
    {
        // Loop through rows
        for (int y = 0; y < image.rows; y++) {
            // Get pointer to start of row
            uint8_t *rowPtr = image.ptr(y);

            // Rotate row to left by m_ScanStep pixels
            std::rotate(rowPtr, rowPtr + m_ScanStep, rowPtr + image.cols);
        }
    }
};
} // Navigation
} // BoBRobotics