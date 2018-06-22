#pragma once

// C++ includes
#include <stdexcept>

// OpenCV
#include <opencv2/opencv.hpp>

// local includes
#include "input.h"

#define PIXPRO_USB_DEVICE_NAME "PIXPRO SP360 4K"
#define WEBCAM360_DEVICE_NAME "USB 2.0 Camera"

namespace BoBRobotics {
namespace Video {
class OpenCVInput : public Input
{
public:
    OpenCVInput()
      : OpenCVInput(0)
    {}

    template<class T>
    OpenCVInput(T dev, const cv::Size &outSize,
                const std::string &cameraName = DefaultCameraName)
      : OpenCVInput(dev, cameraName)
    {
        setOutputSize(outSize);
    }

    template<class T>
    OpenCVInput(T dev, const std::string &cameraName = DefaultCameraName)
      : m_Device(dev), m_CameraName(cameraName)
    {
    }

    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual const std::string getCameraName() const override
    {
        return m_CameraName;
    }

    virtual cv::Size getOutputSize() const override
    {
        cv::Size outSize;
        outSize.width = (int)m_Device.get(cv::CAP_PROP_FRAME_WIDTH);
        outSize.height = (int)m_Device.get(cv::CAP_PROP_FRAME_HEIGHT);
        return outSize;
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        // Try to read next frame
        if (!m_Device.read(outFrame)) {
            throw std::runtime_error("Could not read frame");
        }

        // If there's no error, then we have updated frame and so return true        
        return true;
    }

    virtual void setOutputSize(const cv::Size &outSize) override
    {
        m_Device.set(cv::CAP_PROP_FRAME_WIDTH, outSize.width);
        m_Device.set(cv::CAP_PROP_FRAME_HEIGHT, outSize.height);
    }

private:
    cv::VideoCapture m_Device;
    std::string m_CameraName;
}; // OpenCVInput
} // Video
} // BoBRobotics
