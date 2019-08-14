// BoB robotics includes
#include "video/opencvinput.h"

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
namespace Video {

OpenCVInput::OpenCVInput()
    : OpenCVInput(0)
{}

//------------------------------------------------------------------------
// Video::Input virtuals
//------------------------------------------------------------------------
std::string OpenCVInput::getCameraName() const
{
    return m_CameraName;
}

cv::Size OpenCVInput::getOutputSize() const
{
    cv::Size outSize;
    outSize.width = (int)m_Device.get(cv::CAP_PROP_FRAME_WIDTH);
    outSize.height = (int)m_Device.get(cv::CAP_PROP_FRAME_HEIGHT);
    return outSize;
}

bool OpenCVInput::readFrame(cv::Mat &outFrame)
{
    // Try to read next frame
    if (!m_Device.read(outFrame)) {
        throw std::runtime_error("Could not read frame");
    }

    // If there's no error, then we have updated frame and so return true
    return true;
}

void OpenCVInput::setOutputSize(const cv::Size &outSize)
{
    m_Device.set(cv::CAP_PROP_FRAME_WIDTH, outSize.width);
    m_Device.set(cv::CAP_PROP_FRAME_HEIGHT, outSize.height);
}

} // Video
} // BoBRobotics
