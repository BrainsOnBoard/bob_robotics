#pragma once

// BoB robotics includes
#include "../imgproc/opencv_unwrap_360.h"
#include "input.h"

namespace BoBRobotics {
namespace Video {
class UnwrappedInput
  : public Input
{
public:
    UnwrappedInput(Input &baseInput, const cv::Size &unwrappedSize)
      : m_Unwrapper(baseInput.getOutputSize(), unwrappedSize, baseInput.getCameraName())
      , m_BaseInput(baseInput)
    {}

    virtual std::string getCameraName() const
    {
        return m_BaseInput.getCameraName() + "_unwrapped";
    }

    virtual bool readGreyscaleFrame(cv::Mat &outFrame)
    {
        if (m_BaseInput.readGreyscaleFrame(m_IntermediateFrame)) {
            m_Unwrapper.unwrap(m_IntermediateFrame, outFrame);
            return true;
        } else {
            return false;
        }
    }

    virtual bool needsUnwrapping() const
    {
        return false;
    }

    virtual cv::Size getOutputSize() const
    {
        return m_Unwrapper.getOutputSize();
    }

    virtual bool readFrame(cv::Mat &outFrame)
    {
        if (m_BaseInput.readFrame(m_IntermediateFrame)) {
            m_Unwrapper.unwrap(m_IntermediateFrame, outFrame);
            return true;
        } else {
            return false;
        }
    }

    const auto &getUnwrapper() const
    {
        return m_Unwrapper;
    }

private:
    const ImgProc::OpenCVUnwrap360 m_Unwrapper;
    Input &m_BaseInput;
    cv::Mat m_IntermediateFrame;
};
} // Video
} // BoBRobotics
