#pragma once

// C++ includes
#include <stdexcept>

// OpenCV
#include <opencv2/opencv.hpp>

// local includes
#include "input.h"

#define PIXPRO_USB_DEVICE_NAME "PIXPRO SP360 4K"
#define WEBCAM360_DEVICE_NAME "USB 2.0 Camera"

namespace GeNNRobotics {
namespace Video {
class OpenCVInput
  : public cv::VideoCapture
  , public Input
{
public:
    OpenCVInput()
      : OpenCVInput(0)
    {}

    template<class T>
    OpenCVInput(T dev,
                const cv::Size &outSize,
                const std::string &cameraName = DefaultCameraName)
      : OpenCVInput(dev, cameraName)
    {
        setOutputSize(outSize);
    }

    template<class T>
    OpenCVInput(T dev, const std::string &cameraName = DefaultCameraName)
      : cv::VideoCapture(dev)
    {
        m_CameraName = cameraName;
    }

    const std::string getCameraName() const override
    {
        return m_CameraName;
    }

    cv::Size getOutputSize() const override
    {
        cv::Size outSize;
        outSize.width = (int) get(cv::CAP_PROP_FRAME_WIDTH);
        outSize.height = (int) get(cv::CAP_PROP_FRAME_HEIGHT);
        return outSize;
    }

    bool readFrame(cv::Mat &outFrame) override
    {
        // Read frame
        (*this) >> outFrame;

        // An empty frame means an error has occurred
        if (outFrame.cols == 0) {
            throw std::runtime_error("Could not read frame");
        }

        // If there's no error, then we have updated frame and so return true        
        return true;
    }

    void setOutputSize(const cv::Size &outSize) override
    {
        set(cv::CAP_PROP_FRAME_WIDTH, outSize.width);
        set(cv::CAP_PROP_FRAME_HEIGHT, outSize.height);
    }

protected:
    std::string m_CameraName;
}; // OpenCVInput
} // Video
} // GeNNRobotics
