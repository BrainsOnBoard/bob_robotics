#pragma once

#include <opencv2/opencv.hpp>

#include "videoinput.h"

#define PIXPRO_USB_DEVICE_NAME "PIXPRO SP360 4K"
#define WEBCAM360_DEVICE_NAME "USB 2.0 Camera"

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
                const std::string &cameraName = "unknown_camera")
      : OpenCVInput(dev)
    {
        setOutputSize(outSize);
        m_CameraName = cameraName;
    }

    template<class T>
    OpenCVInput(T dev)
      : cv::VideoCapture(dev)
    {}

    const std::string getCameraName() const
    {
        return m_CameraName;
    }

    cv::Size getOutputSize() const
    {
        cv::Size outSize;
        outSize.width = (int) get(cv::CAP_PROP_FRAME_WIDTH);
        outSize.height = (int) get(cv::CAP_PROP_FRAME_HEIGHT);
        return outSize;
    }

    bool readFrame(cv::Mat &outFrame)
    {
        (*this) >> outFrame;
        return outFrame.cols != 0;
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
