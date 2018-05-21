#pragma once

// C++ includes
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>

// local includes
#include "input.h"
#include "opencvinput.h"
#ifndef _WIN32
#include "../os/video.h"
#include "see3cam_cu40.h"
#endif

namespace GeNNRobotics {
namespace Video {
class PanoramicCamera : public Input
{
public:
    PanoramicCamera()
    {
#ifdef _WIN32
        // for Windows we currently just select the first camera
        m_Camera = std::unique_ptr<Input>(
                new OpenCVInput(0, cv::Size(1280, 720), "webcam360"));
#else
        // get vector of video input devices on system
        auto cameras = OS::Video::getCameras();

        // list of preferred cameras in priority order
        static const std::array<std::string, 3> prefCameras{
            SEE3CAM_DEVICE_NAME, PIXPRO_USB_DEVICE_NAME, WEBCAM360_DEVICE_NAME
        };

        int deviceNum = -1, prefCamNum = -1;
        size_t lowestIndex = prefCameras.size();
        for (OS::Video::CameraDevice cam : cameras) {
            // Iterate through prefCameras looking for matches
            for (size_t i = 0; i < lowestIndex; i++) {
                if (cam.name == prefCameras[i]) {
                    deviceNum = cam.number;
                    prefCamNum = i;
                    lowestIndex = i;
                    break;
                }
            }

            // If we matched the most preferred camera then we're done
            if (prefCamNum == 0) {
                break;
            }

            /*
             * Even if we didn't match any preferred camera, at least this is a
             * camera, so use as fallback
             */
            if (deviceNum == -1) {
                deviceNum = cam.number;
            }
        }
        std::cout << "Selected camera #" << deviceNum << ": "
                  << OS::Video::getCameraName(deviceNum) << std::endl;

        if (deviceNum == -1) {
            throw std::runtime_error("No camera found");
        }

        Input *cam;
        switch (prefCamNum) {
        case 0: // SeeCam
            cam = new See3CAM_CU40("/dev/video" + std::to_string(deviceNum),
                                   See3CAM_CU40::Resolution::_1280x720,
                                   cv::Size(640, 360),
                                   20);
            break;
        case 1: // PixPro
            cam = new OpenCVInput(
                    deviceNum, cv::Size(1440, 1440), "pixpro_usb");
            break;
        default: // webcam with panoramic lens
            cam = new OpenCVInput(deviceNum, cv::Size(1280, 720), "webcam360");
        }
        m_Camera = std::unique_ptr<Input>(cam);
#endif
    }

    void setOutputSize(const cv::Size &outSize)
    {
        m_Camera->setOutputSize(outSize);
    }

    void createDefaultUnwrapper(ImgProc::OpenCVUnwrap360 &unwrapper)
    {
        m_Camera->createDefaultUnwrapper(unwrapper);
    }

    bool readFrame(cv::Mat &outFrame)
    {
        return m_Camera->readFrame(outFrame);
    }

    const std::string getCameraName() const
    {
        return m_Camera->getCameraName();
    }

    virtual cv::Size getOutputSize() const
    {
        return m_Camera->getOutputSize();
    }

private:
    std::unique_ptr<Input> m_Camera;
}; // PanoramicCamera
} // Video
} // GeNNRobotics
