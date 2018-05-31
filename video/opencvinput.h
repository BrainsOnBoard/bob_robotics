#pragma once

#include <opencv2/opencv.hpp>

#include "input.h"

#define PIXPRO_USB_DEVICE_NAME "PIXPRO SP360 4K"
#define WEBCAM360_DEVICE_NAME "USB 2.0 Camera"

namespace GeNNRobotics {
namespace Video {
class OpenCVInput : public Input
{
public:
    OpenCVInput()
      : OpenCVInput(0)
    {}

    template<class T>
    OpenCVInput(T dev, const cv::Size &outSize,
                const std::string &cameraName = "unknown_camera")
      : OpenCVInput(dev, cameraName)
    {
        setOutputSize(outSize);
    }

    template<class T>
    OpenCVInput(T dev, const std::string &cameraName = "unknown_camera")
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
        return m_Device.read(outFrame);
    }

    virtual bool readGreyscaleFrame(cv::Mat &outFrame) override
    {
        // If reading (RGB frame) was succesful
        if(m_Device.read(m_IntermediateFrame)) {
            // If output frame isn't correct size, create it
            if(outFrame.size() != m_IntermediateFrame.size()) {
                outFrame.create(m_IntermediateFrame.size(), CV_8UC1);
            }

            // Convert intermediate frame to greyscale
            cv::cvtColor(m_IntermediateFrame, outFrame, CV_BGR2GRAY);
            return true;
        }
        else {
            return false;
        }
    }

    virtual void setOutputSize(const cv::Size &outSize) override
    {
        m_Device.set(cv::CAP_PROP_FRAME_WIDTH, outSize.width);
        m_Device.set(cv::CAP_PROP_FRAME_HEIGHT, outSize.height);
    }

private:
    cv::Mat m_IntermediateFrame;
    cv::VideoCapture m_Device;
    std::string m_CameraName;
}; // OpenCVInput
} // Video
} // GeNNRobotics
