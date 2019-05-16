// BoB robotics includes
#include "imgproc/opencv_optical_flow.h"
#include "common/macros.h"

// Standard C includes
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/superres/optical_flow.hpp>

namespace BoBRobotics {
namespace ImgProc {
//----------------------------------------------------------------------------
// OpenCVOpticalFlow
//----------------------------------------------------------------------------

OpenCVOpticalFlow::OpenCVOpticalFlow()
  : m_Frame(0)
  , m_OpticalFlow(cv::superres::createOptFlow_Farneback())
{
}

OpenCVOpticalFlow::OpenCVOpticalFlow(const cv::Size &inputRes)
  : OpenCVOpticalFlow()
{
    create(inputRes);
}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
void
OpenCVOpticalFlow::create(const cv::Size &inputRes)
{
    // Create two grayscale frames to hold optical flow
    m_Frames[0].create(inputRes.height, inputRes.width, CV_8UC1);
    m_Frames[1].create(inputRes.height, inputRes.width, CV_8UC1);

    // Create optical flow filter
    m_OpticalFlow = cv::superres::createOptFlow_Farneback();
}

bool
OpenCVOpticalFlow::calculate(const cv::Mat &input)
{
    BOB_ASSERT(input.cols == m_Frames[0].cols);
    BOB_ASSERT(input.rows == m_Frames[0].rows);
    BOB_ASSERT(input.type() == CV_8UC1);

    // Copy frame into array
    const unsigned int currentFrame = m_Frame % 2;
    input.copyTo(m_Frames[currentFrame]);

    // If this isn't the first frame
    if (m_Frame > 0) {
        // Calculate optical flow
        const unsigned int prevFrame = (m_Frame - 1) % 2;
        m_OpticalFlow->calc(m_Frames[prevFrame], m_Frames[currentFrame], m_FlowX, m_FlowY);

        // Increment frame count
        m_Frame++;
        return true;
    } else {
        // Increment frame count
        m_Frame++;

        return false;
    }
}

void
OpenCVOpticalFlow::render(cv::Mat &outputImage, int scale)
{
    BOB_ASSERT(outputImage.cols == m_FlowX.cols * scale);
    BOB_ASSERT(outputImage.rows == m_FlowX.rows * scale);

    // Clear image
    outputImage.setTo(cv::Scalar::all(0));

    // Loop through output coordinates
    for (int x = 0; x < m_FlowX.cols; x++) {
        for (int y = 0; y < m_FlowX.rows; y++) {
            // Draw line showing direction of optical flow
            const cv::Point start(x * scale, y * scale);
            const cv::Point end = start + cv::Point((int) roundf((float) scale * m_FlowX.at<float>(y, x)),
                                                    (int) roundf((float) scale * m_FlowY.at<float>(y, x)));
            cv::line(outputImage, start, end, CV_RGB(0xFF, 0xFF, 0xFF));
        }
    }
}

const cv::Mat &
OpenCVOpticalFlow::getFlowX() const
{
    return m_FlowX;
}
const cv::Mat &
OpenCVOpticalFlow::getFlowY() const
{
    return m_FlowY;
}

} // ImgProc
} // BoBRobotics
