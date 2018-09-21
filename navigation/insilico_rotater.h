#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace Navigation {
class InSilicoRotater
{
public:
    InSilicoRotater(const cv::Size &unwrapRes,
                    const cv::Mat &image,
                    const unsigned int scanStep = 1)
      : m_CurrentRotation(0)
      , m_ScanStep(scanStep)
    {
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        image.copyTo(m_ScratchImage);
    }

    cv::Mat &getImage()
    {
        return m_ScratchImage;
    }

    bool next()
    {
        int next = m_CurrentRotation + m_ScanStep;
        if (next >= m_ScratchImage.cols) {
            return false;
        }

        // Loop through rows
        for (int y = 0; y < m_ScratchImage.rows; y++) {
            // Get pointer to start of row
            uint8_t *rowPtr = m_ScratchImage.ptr(y);

            // Rotate row to left by m_ScanStep pixels
            std::rotate(rowPtr, rowPtr + m_ScanStep, rowPtr + m_ScratchImage.cols);
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
    cv::Mat m_ScratchImage;
};
} // Navigation
} // BoBRobotics