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
      : m_CurrentRotation(0)
      , m_ScanStep(scanStep)
    {
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        image.copyTo(m_ScratchImage);
        maskImage.copyTo(m_ScratchMaskImage);
    }

    cv::Mat &getImage()
    {
        return m_ScratchImage;
    }

    cv::Mat &getMaskImage()
    {
        return m_ScratchMaskImage;
    }

    bool next()
    {
        int next = m_CurrentRotation + m_ScanStep;
        if (next >= m_ScratchImage.cols) {
            return false;
        }

        rollImage(m_ScratchImage);
        if (!m_ScratchMaskImage.empty()) {
            rollImage(m_ScratchMaskImage);
        }
        m_CurrentRotation = next;

        return true;
    }

    size_t count() const
    {
        return m_CurrentRotation / m_ScanStep;
    }

    size_t max() const
    {
        return m_ScratchImage.cols / m_ScanStep;
    }

private:
    int m_CurrentRotation;
    const unsigned int m_ScanStep;
    cv::Mat m_ScratchImage, m_ScratchMaskImage;

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