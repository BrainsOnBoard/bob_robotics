#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Video {
constexpr const char *PixProUSBDeviceName = "PIXPRO SP360 4K";
constexpr const char *Webcam360DeviceName = "USB 2.0 Camera";

//----------------------------------------------------------------------------
// BoBRobotics::Video::OpenCVInput
//----------------------------------------------------------------------------
//! A thin wrapper for reading from any video source supported by OpenCV
class OpenCVInput : public Input
{
public:
    //! Create a new video stream, using the default given by OpenCV
    OpenCVInput();

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
    {
        BOB_ASSERT(m_Device.isOpened());
    }

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
    virtual std::string getCameraName() const override;
    virtual cv::Size getOutputSize() const override;
    virtual bool readFrame(cv::Mat &outFrame) override;
    virtual void setOutputSize(const cv::Size &outSize) override;

private:
    cv::VideoCapture m_Device;
    std::string m_CameraName;
}; // OpenCVInput
} // Video
} // BoBRobotics
