#pragma once

#include "videoinput.h"
#include <opencv2/opencv.hpp>

namespace VideoIn {
class OpenCVInput
  : public cv::VideoCapture
  , public VideoInput
{
public:
    OpenCVInput()
      : OpenCVInput(0)
    {}

    template<class T>
    OpenCVInput(T dev,
                const cv::Size &outSize,
                const std::string &cameraName = "webcam")
      : OpenCVInput(dev)
    {
        setOutputSize(outSize);
        m_CameraName = "webcam";
    }

    template<class T>
    OpenCVInput(T dev)
      : cv::VideoCapture(dev)
    {}

    bool readFrame(cv::Mat &outFrame)
    {
        (*this) >> outFrame;
        return outFrame.cols != 0;
    }

    const std::string getCameraName()
    {
        return m_CameraName;
    }

    void setOutputSize(const cv::Size &outSize)
    {
        set(cv::CAP_PROP_FRAME_WIDTH, outSize.width);
        set(cv::CAP_PROP_FRAME_HEIGHT, outSize.height);
    }

protected:
    std::string m_CameraName;
};
}
