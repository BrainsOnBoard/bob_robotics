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
//----------------------------------------------------------------------------
// BoBRobotics::Video::OpenCVInput
//----------------------------------------------------------------------------
//! A thin wrapper for reading from any video source supported by OpenCV
class OpenCVInput : public Input
{
public:
    //! Create a new video stream, using the default given by OpenCV
    OpenCVInput()
      : OpenCVInput(0)
    {}

    /*!
     * \brief Create a video stream for a specific device
     *
     * @param device Integer or string representation of device (passed to
     *        cv::VideoCapture's constructor)
     * @param cameraName The short name to use for this camera (see getCameraName())
     */
    template<class T>
    OpenCVInput(T device, const std::string &cameraName = DefaultCameraName)
      : m_Device(device), m_CameraName(cameraName)
    {}

    /*!
     * \brief Create a video stream for a specific device and a specified resolution
     *
     * @param device Integer or string representation of device (passed to
     *        cv::VideoCapture's constructor)
     * @param outSize Output resolution of camera
     * @param cameraName The short name to use for this camera (see getCameraName())
     */
    template<class T>
    OpenCVInput(T device, const cv::Size &outSize,
                const std::string &cameraName = DefaultCameraName)
      : OpenCVInput(device, cameraName)
    {
        setOutputSize(outSize);
    }

    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override
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
